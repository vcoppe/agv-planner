use std::{fs::File, io::Read, collections::HashMap};
use cbs::{Graph, CbsNode};

use map::{xml_to_graph, EdgeData, NodeData};
use agv::json_to_agv;
use input::json_to_input;
use nannou_egui::{Egui, egui};
use world::SimpleState;

mod input;
mod map;
mod agv;
mod world;

use std::{sync::Arc, time::Instant};

use cbs::{
    CbsConfig, ConflictBasedSearch, GraphEdgeId, GraphNodeId, MyTime, ReverseResumableAStar,
    Task,
};
use nannou::prelude::*;
use ordered_float::OrderedFloat;

use crate::world::{SimpleWorld, SimpleHeuristic};

fn read_from_file(filename: &str) -> Result<String, std::io::Error> {
    let mut file = File::open(filename)?;
    let mut contents = String::new();
    file.read_to_string(&mut contents)?;
    Ok(contents)
}

struct Model {
    graph: Arc<Graph<NodeData, EdgeData>>,
    mapping: HashMap<GraphNodeId, usize>,
    start_time: f32,
    colors: Vec<rgb::Rgb<nannou::color::encoding::Srgb, u8>>,
    limits: ((f32, f32), (f32, f32)),
    scale: f32,
    window: Rect,
    egui: Option<Egui>,
    index: usize,
    nodes: Vec<Arc<CbsNode<SimpleState, GraphEdgeId, MyTime, MyTime>>>,
    cbs: ConflictBasedSearch<SimpleWorld, SimpleState, GraphEdgeId, MyTime, MyTime, SimpleHeuristic>,
    config: CbsConfig<SimpleWorld, SimpleState, GraphEdgeId, MyTime, MyTime, SimpleHeuristic>,
}

fn main() {
    if true {
        nannou::app(model).update(update).run();
    } else {
        let mut model = get_model();
        let start = Instant::now();
        let solution = model.cbs.solve(&model.config);
        let duration = start.elapsed();

        if let Some(solution) = &solution {
            println!(
                "Solution cost: {}",
                solution.iter().map(|sol| sol.cost).sum::<MyTime>().0
            );
        } else {
            println!("No solution found");
        }

        println!("{:?}", model.cbs.get_stats());
        println!("Time elapsed: {:?}", duration);
    }
}

fn get_model() -> Model {
    let filename = "resources/m.xml";
    let (graph, mapping, inverse_mapping) = xml_to_graph(&read_from_file(filename).unwrap());
    let graph = Arc::new(graph);

    let limits = (0..graph.num_nodes())
        .map(|id| {
            let node = graph.get_node(GraphNodeId(id));
            (node.data.x, node.data.y)
        })
        .fold(
            ((f32::MAX, f32::MAX), (f32::MIN, f32::MIN)),
            |((min_x, min_y), (max_x, max_y)), (x, y)| {
                (
                    (min_x.min(x), min_y.min(y)),
                    (max_x.max(x), max_y.max(y)),
                )
            },
        );

    let filename = "resources/AGVfactsheet.json";
    let agv = json_to_agv(&read_from_file(filename).unwrap());

    let transition_system = Arc::new(SimpleWorld::new(graph.clone(), agv));

    let filename = "resources/t.json";
    let input = json_to_input(&read_from_file(filename).unwrap());
    let mut tasks = vec![];
    for (from, to) in input.tasks.iter().map(|t| (t.start_id, t.goal_id))
    {
        let from = mapping[&from];
        let to = mapping[&to];
        tasks.push(Arc::new(Task::new(
            SimpleState(from),
            SimpleState(to),
            OrderedFloat(0.0),
        )));
    }

    // A set of 20 colors that are visually distinct
    let colors = vec![
        BLUE, GREEN, RED, GOLD, HOTPINK, ORANGE, PURPLE, TEAL, YELLOW, CYAN, PINK, LIME, MAROON,
        NAVY, OLIVE, LAVENDER, BROWN, BEIGE, CORAL, GREY, MAGENTA, TURQUOISE,
    ];

    let pivots = Arc::new(tasks.iter().map(|t| t.goal_state.clone()).collect());
    let heuristic_to_pivots = Arc::new(
        tasks
            .iter()
            .map(|t| {
                Arc::new(ReverseResumableAStar::new(
                    transition_system.clone(),
                    t.clone(),
                    SimpleHeuristic::new(transition_system.clone(), Arc::new(t.reverse())),
                ))
            })
            .collect(),
    );

    let config = CbsConfig::new(tasks, pivots, heuristic_to_pivots, OrderedFloat(1e-6));
    let cbs = ConflictBasedSearch::new(transition_system);

    Model {
        graph,
        mapping: inverse_mapping,
        start_time: 0.0,
        colors,
        scale: 10.0,
        limits,
        window: Rect::from_w_h(1000.0, 1000.0),
        egui: None,
        nodes: vec![],
        index: 0,
        cbs,
        config,
    }
}

fn model(app: &App) -> Model {
    let mut model = get_model();

    let window_id = app.new_window()
        .view(view)
        .raw_event(raw_window_event)
        .build()
        .unwrap();
    let window = app.window(window_id).unwrap();
    model.egui = Some(Egui::from_window(&window));

    let window = app.window_rect().pad(200.0);
    model.scale = (window.w() / (model.limits.1.0 - model.limits.0.0)).min(window.h() / (model.limits.1.1 - model.limits.0.1));
    model.window = window;

    model.start_time = app.time;

    model
}

fn get_node_at(
    model: &mut Model,
    index: usize,
) -> Option<Arc<CbsNode<SimpleState, GraphEdgeId, MyTime, MyTime>>> {
    while index >= model.nodes.len() {
        if let Some(node) = model.cbs.solve_iter(&model.config) {
            model.nodes.push(node);
        } else {
            return None;
        }
    }
    Some(model.nodes[index].clone())
}

fn update(app: &App, model: &mut Model, update: Update) {
    model
        .egui
        .as_mut()
        .unwrap()
        .set_elapsed_time(update.since_start);

    let ctx = model.egui.as_mut().unwrap().begin_frame().clone();
    egui::Window::new("Settings").show(&ctx, |ui| {
        let prev = ui.button("Previous CBS node").clicked();
        if prev {
            model.index = model.index.saturating_sub(1);
        }

        let next = ui.button("Next CBS node").clicked();
        if next {
            model.index = model.index.saturating_add(1);
        }

        if get_node_at(model, model.index).is_none() {
            model.index = model.nodes.len() - 1;
        }

        let first_sol = ui.button("First solution").clicked();
        if first_sol {
            let start = model.index;
            let mut found = false;
            while let Some(node) = get_node_at(model, model.index + 1) {
                model.index += 1;
                if node.conflicts.is_empty() {
                    found = true;
                    break;
                }
            }
            if !found {
                model.index = start;
            }
        }

        if prev || next || first_sol {
            model.start_time = app.time;
        }
    });
}

fn raw_window_event(_app: &App, model: &mut Model, event: &nannou::winit::event::WindowEvent) {
    model.egui.as_mut().unwrap().handle_raw_event(event);
}

fn view(app: &App, model: &Model, frame: Frame) {
    let draw = app.draw();
    draw.background().color(WHITE);

    let to_coordinate = |node: GraphNodeId| {
        let node = model.graph.get_node(node);
        // map node coordinates to window coordinates
        vec2(
            (node.data.x - (model.limits.0.0 + model.limits.1.0) / 2.0) * model.scale,
            (node.data.y - (model.limits.0.1 + model.limits.1.1) / 2.0) * model.scale,
        )
    };

    // Draw graph
    for id in 0..model.graph.num_nodes() {
        draw.ellipse()
            .color(BLACK)
            .radius(2.0)
            .xy(to_coordinate(GraphNodeId(id)));
        draw.text(model.mapping[&GraphNodeId(id)].to_string().as_str())
            .color(BLACK)
            .font_size(14)
            .align_text_middle_y()
            .xy(to_coordinate(GraphNodeId(id)) + vec2(model.scale, model.scale) / 4.0);
    }

    for id in 0..model.graph.num_edges() {
        let edge = model.graph.get_edge(GraphEdgeId(id));
        draw.arrow()
            .color(BLACK)
            .start(to_coordinate(edge.from))
            .end(to_coordinate(edge.to));
    }

    // Draw agents
    let elapsed = app.time - model.start_time;
    let mut current_time = OrderedFloat(elapsed);

    let solutions = model.nodes[model.index].get_solutions(model.config.n_agents);

    let max_time = solutions
        .iter()
        .map(|solution| solution.cost)
        .max()
        .unwrap();

    while current_time > max_time {
        current_time = current_time - max_time;
    }

    for (agent, solution) in solutions.iter().enumerate() {
        let mut drawn = false;
        for i in 0..(solution.steps.len() - 1) {
            if current_time >= solution.steps[i].1 && current_time <= solution.steps[i + 1].1 {
                let from = to_coordinate(solution.steps[i].0.internal_state.0);
                let to = to_coordinate(solution.steps[i + 1].0.internal_state.0);

                let delta = to - from;
                let progress_time = current_time - solution.steps[i].1;
                let move_time = solution.steps[i + 1].1 - solution.steps[i].1;
                let vel = delta / move_time.0;
                let center = from + vel * progress_time.0;

                draw.ellipse()
                    .color(model.colors[agent])
                    .radius(0.4 * model.scale)
                    .xy(center);
                draw.text(agent.to_string().as_str())
                    .color(WHITE)
                    .font_size(18)
                    .xy(center);
                drawn = true;
                break;
            }
        }

        if !drawn {
            let center = to_coordinate(solution.steps.last().unwrap().0.internal_state.0);
            draw.ellipse()
                .color(model.colors[agent])
                .radius(0.4 * model.scale)
                .xy(center);
            draw.text(agent.to_string().as_str())
                .color(WHITE)
                .font_size(18)
                .align_text_middle_y()
                .xy(center);
        }
    }

    let mut text = String::new();
    let mut constraint_texts = vec![];

    text += &format!("Current cost: {}\n", model.nodes[model.index].total_cost.0);

    // Draw constraints
    for agent in 0..model.config.n_agents {
        text += &format!("Constraints for agent {}:\n", agent);

        let (constraints, landmarks) = model.nodes[model.index].get_constraints(agent);
        for ((from, to), constraint_set) in constraints.action_constraints.iter() {
            for constraint in constraint_set {
                if current_time >= constraint.interval.start
                    && current_time <= constraint.interval.end
                {
                    let from = to_coordinate(from.0);
                    let to = to_coordinate(to.0);

                    let delta = to - from;
                    let delta = delta.rotate(0.5 * PI).clamp_length_max(model.scale / 10.0);

                    let from = from + delta;
                    let to = to + delta;

                    draw.line()
                        .color(rgba8(
                            model.colors[agent].red,
                            model.colors[agent].green,
                            model.colors[agent].blue,
                            120,
                        ))
                        .start(from)
                        .end(to)
                        .weight(model.scale / 5.0);
                }

                constraint_texts.push(format!(
                    "- Action constraint between nodes {} and {}, between {} and {}\n",
                    from.0 .0, to.0 .0, constraint.interval.start.0, constraint.interval.end.0
                ));
            }
        }

        constraint_texts.sort_unstable();
        text += &constraint_texts.join("");
        constraint_texts.clear();

        for (state, constraint_set) in constraints.state_constraints.iter() {
            for constraint in constraint_set {
                if current_time >= constraint.interval.start
                    && current_time <= constraint.interval.end
                {
                    draw.rect()
                        .w_h(model.scale, model.scale)
                        .color(rgba8(
                            model.colors[agent].red,
                            model.colors[agent].green,
                            model.colors[agent].blue,
                            120,
                        ))
                        .xy(to_coordinate(state.0));
                }

                constraint_texts.push(format!(
                    "- State constraint at node {}, between {} and {}\n",
                    state.0 .0, constraint.interval.start.0, constraint.interval.end.0
                ));
            }
        }

        constraint_texts.sort_unstable();
        text += &constraint_texts.join("");
        constraint_texts.clear();

        for constraint in landmarks.iter() {
            if current_time >= constraint.interval.start && current_time <= constraint.interval.end
            {
                draw.ellipse()
                    .radius(model.scale / 2.0)
                    .no_fill()
                    .stroke(rgba8(
                        model.colors[agent].red,
                        model.colors[agent].green,
                        model.colors[agent].blue,
                        120,
                    ))
                    .stroke_weight(model.scale / 10.0)
                    .xy(to_coordinate(constraint.state.0));
            }

            constraint_texts.push(format!(
                "- Landmark at node {}, between {} and {}\n",
                constraint.state.0 .0, constraint.interval.start.0, constraint.interval.end.0
            ));
        }
    }

    constraint_texts.sort_unstable();
    text += &constraint_texts.join("");
    constraint_texts.clear();

    text += &format!("Remaining conflicts\n");
    for conflict in model.nodes[model.index].conflicts.iter() {
        text += &format!(
            "- Agent {} moving between nodes {} and {}, between {} and {}, and agent {} moving between {} and {}, between {} and {} ({:?})\n",
            conflict.moves.0.agent,
            conflict.moves.0.from.0 .0,
            conflict.moves.0.to.0 .0,
            conflict.moves.0.interval.start.0,
            conflict.moves.0.interval.end.0,
            conflict.moves.1.agent,
            conflict.moves.1.from.0 .0,
            conflict.moves.1.to.0 .0,
            conflict.moves.1.interval.start.0,
            conflict.moves.1.interval.end.0,
            conflict.type_,
        );
    }

    draw.text(text.as_str())
        .left_justify()
        .color(BLACK)
        .font_size(12)
        .xy(vec2(model.window.w() * 0.7, 0.0))
        .width(model.scale * 5.0);

    draw.to_frame(app, &frame).unwrap();
    model.egui.as_ref().unwrap().draw_to_frame(&frame).unwrap();
}

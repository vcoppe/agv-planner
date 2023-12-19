use std::{fs::File, io::Read, collections::HashMap};
use cbs::Graph;

use map::{xml_to_graph, EdgeData, NodeData};
use agv::json_to_agv;
use input::json_to_input;
use world::SimpleState;

mod input;
mod map;
mod agv;
mod world;

use std::{sync::Arc, time::Instant};

use cbs::{
    CbsConfig, ConflictBasedSearch, GraphEdgeId, GraphNodeId, MyTime, ReverseResumableAStar, SippState, Solution,
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
    solution:
        Option<Vec<Solution<Arc<SippState<SimpleState, MyTime>>, GraphEdgeId, MyTime, MyTime>>>,
    start_time: f32,
    colors: Vec<rgb::Rgb<nannou::color::encoding::Srgb, u8>>,
    limits: ((f32, f32), (f32, f32)),
    scale: f32,
    window: Rect,
}

fn main() {
    if true {
        nannou::app(model).update(update).run();
    } else {
        get_model();
    }
}

fn get_model() -> Model {
    let filename = "resources/map.xml";
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

    let filename = "resources/input.json";
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

    let mut cbs = ConflictBasedSearch::new(transition_system);

    let start = Instant::now();
    let solution = cbs.solve(&config);
    let duration = start.elapsed();

    if let Some(solution) = &solution {
        println!(
            "Solution cost: {}",
            solution.iter().map(|sol| sol.cost).sum::<MyTime>().0
        );
    } else {
        println!("No solution found");
    }

    println!("{:?}", cbs.get_stats());
    println!("Time elapsed: {:?}", duration);

    Model {
        graph,
        mapping: inverse_mapping,
        solution,
        start_time: 0.0,
        colors,
        scale: 10.0,
        limits,
        window: Rect::from_w_h(1000.0, 1000.0),
    }
}

fn model(app: &App) -> Model {
    let mut model = get_model();

    app.new_window()
        .view(view)
        .build()
        .unwrap();

    let window = app.window_rect().pad(50.0);
    model.scale = (window.w() / (model.limits.1.0 - model.limits.0.0)).min(window.h() / (model.limits.1.1 - model.limits.0.1));
    model.window = window;

    model.start_time = app.time;

    model
}

fn update(_app: &App, _model: &mut Model, _update: Update) {}

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

    let solutions = model.solution.as_ref().unwrap();

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

    draw.to_frame(app, &frame).unwrap();
}

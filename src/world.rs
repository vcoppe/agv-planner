use std::sync::Arc;

use ncollide2d::{
    na::{Point2, Vector2},
    query,
    shape::Ball,
};
use tuple::A2;

use caboose::{
    Graph, GraphEdgeId, GraphNodeId, Move, State,
    TransitionSystem, MyTime, Task, Heuristic, HeuristicBuilder,
};

use crate::{map::{NodeData, EdgeData}, agv::AGV};

pub struct SimpleWorld {
    graph: Arc<Graph<NodeData, EdgeData>>,
    agv: AGV,
}

impl SimpleWorld {
    const BALL: Ball<f64> = Ball { radius: 0.4 };

    pub fn new(graph: Arc<Graph<NodeData, EdgeData>>, agv: AGV) -> Self {
        Self { graph, agv }
    }

    pub fn time_between(&self, from: GraphNodeId, to: GraphNodeId) -> MyTime {
        let from = self.graph.get_node(from);
        let to = self.graph.get_node(to);
        let d_x = to.data.x - from.data.x;
        let d_y = to.data.y - from.data.y;
        ((d_x * d_x + d_y * d_y).sqrt() / self.agv.physical_parameters.speed_max).into()
    }

    pub fn time(&self, edge: GraphEdgeId) -> MyTime {
        let edge = self.graph.get_edge(edge);
        (edge.data.length / self.agv.physical_parameters.speed_max.min(edge.data.max_speed)).into()
    }

    pub fn get_center_and_vel(
        &self,
        m: &Move<SimpleState, GraphEdgeId, MyTime, MyTime>,
        initial_time: &MyTime,
    ) -> (Point2<f64>, Vector2<f64>) {
        let interval = &m.interval;
        let from = &self.graph.get_node(m.from.0).data;
        let to = &self.graph.get_node(m.to.0).data;

        let d_x = to.x - from.x;
        let d_y = to.y - from.y;
        let d_t = interval.end.0 - interval.start.0;
        let vel_x = d_x / d_t;
        let vel_y = d_y / d_t;

        let pre_d_t = initial_time.0 - interval.start.0;

        let center_x = from.x + vel_x * pre_d_t;
        let center_y = from.y + vel_y * pre_d_t;

        (Point2::new(center_x, center_y), Vector2::new(vel_x, vel_y))
    }
}

#[derive(Debug, PartialEq, Eq, Hash, Clone)]
pub struct SimpleState(pub GraphNodeId);

impl State for SimpleState {
    fn is_equivalent(&self, other: &Self) -> bool {
        self.0 == other.0
    }
}

impl TransitionSystem<SimpleState, GraphEdgeId, MyTime, MyTime> for SimpleWorld {
    fn actions_from(&self, state: &SimpleState) -> std::slice::Iter<GraphEdgeId> {
        self.graph.get_edges_out(state.0).iter()
    }

    fn transition(&self, _state: &SimpleState, action: &GraphEdgeId) -> SimpleState {
        SimpleState(self.graph.get_edge(*action).to)
    }

    fn transition_cost(&self, _state: &SimpleState, action: &GraphEdgeId) -> MyTime {
        self.time(*action)
    }

    fn reverse_actions_from(&self, state: &SimpleState) -> std::slice::Iter<GraphEdgeId> {
        self.graph.get_edges_in(state.0).iter()
    }

    fn reverse_transition(&self, _state: &SimpleState, action: &GraphEdgeId) -> SimpleState {
        SimpleState(self.graph.get_edge(*action).from)
    }

    fn reverse_transition_cost(&self, _state: &SimpleState, action: &GraphEdgeId) -> MyTime {
        self.time(*action)
    }

    fn can_wait_at(&self, _state: &SimpleState) -> bool {
        true
    }

    fn conflict(&self, moves: A2<&Move<SimpleState, GraphEdgeId, MyTime, MyTime>>) -> bool {
        let initial_time = moves[0].interval.start.max(moves[1].interval.start);
        let max_time = moves[0].interval.end.min(moves[1].interval.end) - initial_time;

        let (center1, vel1) = self.get_center_and_vel(moves[0], &initial_time);
        let (center2, vel2) = self.get_center_and_vel(moves[1], &initial_time);

        let toi = query::time_of_impact_ball_ball(
            &center1,
            &vel1,
            &Self::BALL,
            &center2,
            &vel2,
            &Self::BALL,
            max_time.0,
            0.0,
        );

        toi.is_some()
    }
}

pub struct SimpleHeuristic {
    transition_system: Arc<SimpleWorld>,
    goal_state: SimpleState,
}

impl SimpleHeuristic {
    pub fn new(transition_system: Arc<SimpleWorld>, task: Arc<Task<SimpleState, MyTime>>) -> Self {
        SimpleHeuristic {
            transition_system,
            goal_state: task.goal_state.clone(),
        }
    }
}

impl Heuristic<SimpleWorld, SimpleState, GraphEdgeId, MyTime, MyTime> for SimpleHeuristic {
    fn get_heuristic(&self, state: &SimpleState) -> Option<MyTime> {
        Some(
            self.transition_system
                .time_between(state.0, self.goal_state.0),
        )
    }
}

impl HeuristicBuilder<SimpleWorld, SimpleState, GraphEdgeId, MyTime, MyTime> for SimpleHeuristic {
    fn build(transition_system: Arc<SimpleWorld>, task: Arc<Task<SimpleState, MyTime>>) -> Self {
        Self::new(transition_system, task)
    }
}
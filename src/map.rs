use std::collections::HashMap;

use cbs::Graph;
use serde::Deserialize;
use quick_xml::de::{from_str, DeError};

#[derive(Debug)]
pub struct NodeData
{
    pub x: f32,
    pub y: f32,
    pub deviation: f32,
}

#[derive(Debug)]
pub struct EdgeData
{
    pub length: f32,
    pub max_speed: f32,
}

#[derive(Debug, Deserialize)]
pub struct Map {
    nodes: Nodes,
    edges: Edges,
}

#[derive(Debug, Deserialize)]
struct Nodes {
    #[serde(rename = "node", default)]
    nodes: Vec<Node>,
}


#[derive(Debug, Deserialize)]
struct Node {
    #[serde(rename = "@id")]
    id: isize,
    #[serde(rename = "nodePosition")]
    node_position: NodePosition,
}

#[derive(Debug, Deserialize)]
struct NodePosition {
    x: f32,
    y: f32,
    #[serde(rename = "allowedDeviationXY")]
    allowed_deviation_xy: f32,
}

#[derive(Debug, Deserialize)]
struct Edges {
    #[serde(rename = "edge", default)]
    edges: Vec<Edge>,
}

#[derive(Debug, Deserialize)]
struct Edge {
    #[serde(rename = "@id")]
    id: isize,
    #[serde(rename = "startNodeId")]
    start_node_id: isize,
    #[serde(rename = "endNodeId")]
    end_node_id: isize,
    #[serde(rename = "maxSpeed")]
    max_speed: f32,
    #[serde(rename = "maxHeight")]
    max_height: f32,
    #[serde(rename = "rotationAllowed")]
    rotation_allowed: bool,
    length: f32,
}

pub fn xml_to_graph(xml: &str) -> Graph<NodeData, EdgeData> {
    let data: Result<Map, DeError>  = from_str(xml);
    let map = match data {
        Ok(map) => map,
        Err(err) => panic!("Error parsing XML: {:?}", err),
    };
    let mut mapping = HashMap::new();
    let mut graph = Graph::new();
    for node in map.nodes.nodes {
        let id = graph.add_node(NodeData {
            x: node.node_position.x,
            y: node.node_position.y,
            deviation: node.node_position.allowed_deviation_xy,
        });
        mapping.insert(node.id, id);
    }
    for edge in map.edges.edges {
        let from = *mapping.get(&edge.start_node_id).unwrap();
        let to = *mapping.get(&edge.end_node_id).unwrap();
        graph.add_edge(from, to, EdgeData {
            length: edge.length,
            max_speed: edge.max_speed,
        });
    }
    graph
}
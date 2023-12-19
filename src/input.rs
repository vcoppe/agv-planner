use serde::Deserialize;
use serde_json::from_str;


#[derive(Debug, Deserialize)]
pub struct Input {
    pub tasks: Vec<Task>,
}

#[derive(Debug, Deserialize)]
pub struct Task {
    pub agent: usize,
    #[serde(rename = "startId")]
    pub start_id: usize,
    #[serde(rename = "goalId")]
    pub goal_id: usize,
}

pub fn json_to_input(json: &str) -> Input {
    let data = from_str(json);
    let data = match data {
        Ok(data) => data,
        Err(err) => panic!("Error parsing JSON: {:?}", err),
    };
    data
}

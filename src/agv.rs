use serde::Deserialize;
use serde_json::from_str;


#[derive(Debug, Deserialize)]
pub struct AGV {
    #[serde(rename = "physicalParameters")]
    pub physical_parameters: PhysicalParameters,
}

#[derive(Debug, Deserialize)]
pub struct PhysicalParameters {
    #[serde(rename = "speedMin")]
    pub speed_min: f32,
    #[serde(rename = "speedMax")]
    pub speed_max: f32,
    #[serde(rename = "accelerationMax")]
    pub acceleration_max: f32,
    #[serde(rename = "decelerationMax")]
    pub deceleration_max: f32,
    #[serde(rename = "heightMin")]
    pub height_min: f32,
    #[serde(rename = "heightMax")]
    pub height_max: f32,
    pub width: f32,
    pub length: f32,
}

pub fn json_to_agv(json: &str) -> AGV {
    let data = from_str(json);
    let agv = match data {
        Ok(agv) => agv,
        Err(err) => panic!("Error parsing JSON: {:?}", err),
    };
    agv
}

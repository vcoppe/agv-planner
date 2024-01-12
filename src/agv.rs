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
    pub speed_min: f64,
    #[serde(rename = "speedMax")]
    pub speed_max: f64,
    #[serde(rename = "accelerationMax")]
    pub acceleration_max: f64,
    #[serde(rename = "decelerationMax")]
    pub deceleration_max: f64,
    #[serde(rename = "heightMin")]
    pub height_min: f64,
    #[serde(rename = "heightMax")]
    pub height_max: f64,
    pub width: f64,
    pub length: f64,
}

pub fn json_to_agv(json: &str) -> AGV {
    let data = from_str(json);
    let agv = match data {
        Ok(agv) => agv,
        Err(err) => panic!("Error parsing JSON: {:?}", err),
    };
    agv
}

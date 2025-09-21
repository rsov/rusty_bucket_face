use alloc::{string::String, vec::Vec};
use defmt::info;
use slint::{ComponentHandle, VecModel, Weak};

use crate::{Api, AppWindow, SignalModel};

#[derive(Debug, serde::Deserialize)]
pub struct CanProfile {
    name: String,
    standard_id: bool,
    messages: Vec<CanMessage>,
}

#[derive(Debug, serde::Deserialize)]
pub struct CanMessage {
    can_id: String,
    signals: Vec<CanSignal>,
}

#[derive(Debug, serde::Deserialize)]
pub struct CanSignal {
    name: String,
    mask: String,
    signed: bool,
    units: String,
    conversion: Option<String>,
    bit: Option<i8>,
}

// TODO: Probably store in memory?
pub fn load_profile() -> CanProfile {
    let json_data = include_str!("./can_profiles/can_profile_haltech.json"); // or read from file
    let profile: CanProfile = serde_json::from_str(json_data).expect("Failed to parse JSON");
    return profile;
}

#[embassy_executor::task]
pub async fn load_signals_to_ui(ui_handle: Weak<AppWindow>) {
    info!("✓ Signals");

    let mut signal_models: Vec<SignalModel> = Vec::new();

    let profile = load_profile();

    for messages in &profile.messages {
        for signal in &messages.signals {
            signal_models.push(SignalModel {
                name: signal.name.clone().into(),
            });
        }
    }

    let ui = ui_handle.upgrade().unwrap();
    ui.global::<Api>()
        .set_signals(VecModel::from_slice(&signal_models));
}

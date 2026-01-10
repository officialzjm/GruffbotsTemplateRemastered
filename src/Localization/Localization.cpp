#include "Localization.h"

Localization::Localization(Odom* odom) {
    this->odom_task = nullptr;
    this->odom = odom;
}

void Localization::StartOdomTracking() {

    if (this->odom_task == nullptr) {
        this->odom_task = new pros::Task(PositionTrackTask, this, "Odom Task");
    }
}

void Localization::PositionTrackTask(void* chassis) {
    Localization* locomotion = static_cast<Localization*>(chassis);
    locomotion->PositionTrack();
}

void Localization::PositionTrack() {
    while (this->odom_task != nullptr && this->odom_task->get_state() != pros::E_TASK_STATE_DELETED) {
        this->odom->Update_Coordinates(); 
        pros::delay(LocalizationConfig::ODOM_UPDATE_DELAY_MS); // Adjust delay as needed
    }
}

void Localization::StopOdomTracking() {
    if (this->odom_task != nullptr) {
        this->odom_task->remove();
        delete this->odom_task;
        this->odom_task = nullptr;
    }
}
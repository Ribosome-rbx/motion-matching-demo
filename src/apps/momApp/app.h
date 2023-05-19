#pragma once

#include <crl-basic/gui/application.h>
#include <imgui_widgets/imfilebrowser.h>
#include <crl-basic/utils/logger.h>


#include "mom/MotionMatching.h"

namespace momApp {

class App : public crl::gui::ShadowApplication {
public:
    App() : crl::gui::ShadowApplication("MotionMatching App") {
        // load mann dataset
        std::string mocapPath = dataPath_ + "walk1_subject5.bvh";
        mocapSkeleton = new crl::mocap::MocapSkeleton(mocapPath.c_str());
        motionDatabase = new crl::mocap::MotionDatabase(dataPath_);
        motionMatching = new crl::mocap::MotionMatching(mocapSkeleton, motionDatabase);
        motionMatching->queueSize = 60;
    }

    ~App() override {
        delete mocapSkeleton;
        delete motionDatabase;
    }

    void process() override {
        static uint frame = 0;
        if (frame >= 30 || NEW_INPUT) {
            crl::Logger::consolePrint("transition happens!");
            motionMatching->matchMotion(camera);
            frame = 0;
            NEW_INPUT = false;
        }
        motionMatching->advance();
        frame++;

        crl::P3D pos_(mocapSkeleton->root->state.pos.x, 0, mocapSkeleton->root->state.pos.z);
        crl::V3D vel_(crl::P3D(mocapSkeleton->root->state.velocity[0], 0, mocapSkeleton->root->state.velocity[2]));
        motionMatching->historyPos.push_back(pos_);
        motionMatching->historyVel.push_back(vel_.normalized());

        camera.target.x = mocapSkeleton->root->state.pos.x;
        camera.target.z = mocapSkeleton->root->state.pos.z;
        light.target.x() = mocapSkeleton->root->state.pos.x;
        light.target.z() = mocapSkeleton->root->state.pos.z;
    }

    void drawShadowCastingObjects(const crl::gui::Shader &shader) override {
        mocapSkeleton->draw(shader);
    }

    void drawObjectsWithoutShadows(const crl::gui::Shader &shader) override {
        mocapSkeleton->draw(shader);
        // motionMatching->drawDebugInfo(shader, camera);

        // customized draw
        drawArrow3d(mocapSkeleton->root->state.pos, motionMatching->goalVel.normalized(), 0.02, shader, crl::V3D(1, 0.55, 0), 0.5);
        drawArrow3d(mocapSkeleton->root->state.pos, motionMatching->cameraDir.normalized(), 0.02, shader, crl::V3D(0, 0.55, 1), 0.5);

        for (int i=0; i<motionMatching->PosTraj.getKnotCount(); i++){
            crl::P3D pos_i = crl::P3D() + motionMatching->PosTraj.evaluate_linear(i / 60.0);
            drawSphere(pos_i, 0.02, shader, crl::V3D(1, 0, 0.2), 0.5);
            if (i > 0){
                crl::V3D vel_i = motionMatching->VelTraj.evaluate_linear(i / 60.0);
                drawArrow3d(pos_i, vel_i.normalized(), 0.02, shader, crl::V3D(1, 0, 1), 0.5);
            }
        }

        int start = motionMatching->historyPos.size() >= 120? motionMatching->historyPos.size()-120 : 0;
        for (int i=start; i<motionMatching->historyPos.size(); i++){
            crl::P3D pos_i = motionMatching->historyPos[i];
            drawSphere(pos_i, 0.02, shader, crl::V3D(0, 0, 1), 0.5);
            if(i % 20 == 19){
                crl::V3D vel_i = (motionMatching->historyVel[i]).normalized();
                drawArrow3d(pos_i, vel_i, 0.02, shader, crl::V3D(0, 0, 1), 0.5);
            }
        }
    }

    void drawImGui() override {
        crl::gui::ShadowApplication::drawImGui();

        ImGui::Begin("Main Menu");
        if (ImGui::CollapsingHeader("Motion Control options", ImGuiTreeNodeFlags_DefaultOpen)) {
            ImGui::InputDouble("Speed forward", &motionMatching->speedForward, 0.0f, 0.0f, "%.3f", ImGuiInputTextFlags_ReadOnly);
            ImGui::InputDouble("Speed turning", &motionMatching->turningSpeed, 0.0f, 0.0f, "%.3f", ImGuiInputTextFlags_ReadOnly);
        }
        ImGui::End();
    }

    virtual bool keyPressed(int key, int mods) override {
        if (key == GLFW_KEY_SPACE) {
            processIsRunning = !processIsRunning;
        }
        if (key == GLFW_KEY_ENTER) {
            if (!processIsRunning)
                process();
        }
        if (key == GLFW_KEY_BACKSPACE) {
            screenIsRecording = !screenIsRecording;
        }
        if (key == GLFW_KEY_UP) {
            motionMatching->speedForward += 0.3;
        }
        if (key == GLFW_KEY_DOWN) {
            motionMatching->speedForward -= 0.3;
        }
        if (key == GLFW_KEY_LEFT) {
            motionMatching->turningSpeed += 0.3;
        }
        if (key == GLFW_KEY_RIGHT) {
            motionMatching->turningSpeed -= 0.3;
        }
        if (key == GLFW_KEY_W) {
            // motionMatching->speedForward = 3.0;
            if (!motionMatching->KEY_W) NEW_INPUT = true;
            motionMatching->KEY_W = true;
        }
        if (key == GLFW_KEY_S) {
            // motionMatching->speedForward = -3.0;
            if (!motionMatching->KEY_S) NEW_INPUT = true;
            motionMatching->KEY_S = true;
        }
        if (key == GLFW_KEY_A) {
            // motionMatching->turningSpeed = 1.0;
            if (!motionMatching->KEY_A) NEW_INPUT = true;
            motionMatching->KEY_A = true;
        }
        if (key == GLFW_KEY_D) {
            // motionMatching->turningSpeed = -1.0;
            if (!motionMatching->KEY_D) NEW_INPUT = true;
            motionMatching->KEY_D = true;
        }

        return false;
    }

    virtual bool keyReleased(int key, int mods) override{
        if (key == GLFW_KEY_W) {
            if (motionMatching->KEY_W) NEW_INPUT = true;
            motionMatching->KEY_W = false;
        }
        if (key == GLFW_KEY_S) {
            if (motionMatching->KEY_S) NEW_INPUT = true;
            motionMatching->KEY_S = false;
        }
        if (key == GLFW_KEY_A) {
            if (motionMatching->KEY_A) NEW_INPUT = true;
            motionMatching->KEY_A = false;
        }
        if (key == GLFW_KEY_D) {
            if (motionMatching->KEY_D) NEW_INPUT = true;
            motionMatching->KEY_D = false;
        }

        return false;
    }

    bool mouseMove(double xpos, double ypos) override {
        if (mouseState.dragging == false) {
            crl::P3D rayOrigin;
            crl::V3D rayDirection;
            camera.getRayFromScreenCoordinates(xpos, ypos, rayOrigin, rayDirection);
        }
        return crl::gui::ShadowApplication::mouseMove(xpos, ypos);
    }

    bool mouseButtonPressed(int button, int mods) override {
        return true;
    }

    bool drop(int count, const char **fileNames) override {
        return true;
    }

public:
    std::string dataPath_ = MOTION_MATCHING_DEMO_DATA_FOLDER "/mocap/lafan1_mini/";
    crl::mocap::MocapSkeleton *mocapSkeleton = nullptr;
    crl::mocap::MotionDatabase *motionDatabase = nullptr;
    crl::mocap::MotionMatching *motionMatching = nullptr;
private:
    bool NEW_INPUT = false;
};

}  // namespace momApp
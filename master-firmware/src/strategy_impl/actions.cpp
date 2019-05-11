#include <aversive/blocking_detection_manager/blocking_detection_manager.h>

#include <aversive/position_manager/position_manager.h>
#include <aversive/trajectory_manager/trajectory_manager_utils.h>
#include <aversive/obstacle_avoidance/obstacle_avoidance.h>

#include "robot_helpers/math_helpers.h"
#include "robot_helpers/trajectory_helpers.h"
#include "robot_helpers/beacon_helpers.h"
#include "robot_helpers/strategy_helpers.h"

#include "control_panel.h"
#include "protobuf/sensors.pb.h"
#include "config.h"

#include "strategy_impl/actions.h"

bool IndexArms::execute(RobotState& state)
{
    strat->log("Indexing arms!");

    // set index when user presses color button, so indexing is done manually

    strat->manipulator_disable(RIGHT);
    strat->manipulator_disable(LEFT);

    strat->gripper_set(RIGHT, GRIPPER_ACQUIRE);
    strat->wait_ms(1000);
    strat->wait_for_user_input(); // wait for user to align the right arm
    strat->arm_manual_index(RIGHT);
    strat->gripper_set(RIGHT, GRIPPER_OFF);

    strat->gripper_set(LEFT, GRIPPER_ACQUIRE);
    strat->wait_ms(1000);
    strat->wait_for_user_input(); // wait for user to align the left arm
    strat->arm_manual_index(LEFT);
    strat->gripper_set(LEFT, GRIPPER_OFF);

    strat->wait_ms(1000);
    strat->wait_for_user_input(); // wait for user to remove the alignment part
    state.arms_are_indexed = true;
    return true;
}

bool RetractArms::execute(RobotState& state)
{
    strat->log("Retracting arms!");

    strat->gripper_set(BOTH, GRIPPER_OFF);
    if (!strat->manipulator_goto(BOTH, MANIPULATOR_STORE_FRONT_2)) {
        strat->manipulator_goto(RIGHT, MANIPULATOR_STORE_FRONT_2);
        strat->manipulator_goto(LEFT, MANIPULATOR_STORE_FRONT_2);
    }

    state.right_has_puck = false;
    state.left_has_puck = false;

    state.arms_are_deployed = false;

    return true;
}

bool TakePuck::execute(RobotState& state)
{
    if (pucks[puck_id].color == PuckColor_RED)
        strat->log("Taking red puck !");
    if (pucks[puck_id].color == PuckColor_GREEN)
        strat->log("Taking green puck !");
    if (pucks[puck_id].color == PuckColor_BLUE)
        strat->log("Taking blue puck !");
    if (pucks[puck_id].color == PuckColor_RED_OR_GREEN)
        strat->log("Taking red/green puck !");
    if (pucks[puck_id].color == PuckColor_GOLDENIUM)
        strat->log("Taking golden puck !");

    strat->log((side == LEFT) ? "\tUsing left arm" : "\tUsing right arm");

    float x, y, a;
    if (pucks[puck_id].orientation == PuckOrientiation_HORIZONTAL) {
        x = MIRROR_X(strat->color, pucks[puck_id].pos_x_mm - 160);
        y = pucks[puck_id].pos_y_mm + MIRROR_ARM(side, MIRROR(strat->color, 55));
        a = MIRROR_A(strat->color, 180);
    } else {
        x = MIRROR_X(strat->color, pucks[puck_id].pos_x_mm) - MIRROR_ARM(side, 55);
        y = pucks[puck_id].pos_y_mm - 220;
        a = MIRROR_A(strat->color, -90);
    }

    if (!strat->goto_xya(strat, x, y, a)) {
        return false;
    }

    state.arms_are_deployed = true;
    strat->gripper_set(side, GRIPPER_ACQUIRE);

    if (pucks[puck_id].orientation == PuckOrientiation_HORIZONTAL) {
        strat->manipulator_goto(side, MANIPULATOR_PICK_HORZ);
        strat->wait_ms(500);
        strat->manipulator_goto(side, MANIPULATOR_STORE_FRONT_0);
        state.arms_are_deployed = false;
    } else {
        strat->manipulator_goto(side, MANIPULATOR_PICK_VERT);
        strat->forward(strat, -30);
        strat->wait_ms(500);
        strat->manipulator_goto(side, MANIPULATOR_LIFT_VERT);
        strat->forward(strat, 60);
        strat->manipulator_goto(side, MANIPULATOR_STORE_FRONT_0);
        state.arms_are_deployed = false;
    }

    state.puck_available[puck_id] = false;

    if (!strat->puck_is_picked(side)) {
        strat->gripper_set(side, GRIPPER_OFF);
        return false;
    }

    if (side == LEFT) {
        state.left_has_puck = true;
        state.left_puck_color = pucks[puck_id].color;
    } else {
        state.right_has_puck = true;
        state.right_puck_color = pucks[puck_id].color;
    }
    return true;
}

bool TakeTwoPucks::execute(RobotState& state)
{
    strat->log("Taking two pucks: blue and green !");

    float x = MIRROR_X(strat->color, 175);
    float y = pucks[puck_id_left].pos_y_mm - 220;
    float a = MIRROR_A(strat->color, -90);

    if (!strat->goto_xya(strat, x, y, a)) {
        return false;
    }

    state.arms_are_deployed = true;
    strat->gripper_set(BOTH, GRIPPER_ACQUIRE);

    strat->manipulator_goto(BOTH, MANIPULATOR_PICK_VERT);
    strat->forward(strat, -30);
    strat->wait_ms(500);
    strat->forward(strat, 100);
    strat->manipulator_goto(BOTH, MANIPULATOR_LIFT_VERT);
    strat->manipulator_goto(BOTH, MANIPULATOR_STORE_FRONT_0);
    state.arms_are_deployed = false;

    state.puck_available[puck_id_left] = false;
    state.puck_available[puck_id_right] = false;

    if (strat->puck_is_picked(RIGHT)) {
        state.right_has_puck = true;
        state.right_puck_color = pucks[puck_id_right].color;
    }
    if (strat->puck_is_picked(LEFT)) {
        state.left_has_puck = true;
        state.left_puck_color = pucks[puck_id_left].color;
    }

    // if no puck taken, fail!
    if (!state.right_has_puck && !state.left_has_puck) {
        strat->gripper_set(BOTH, GRIPPER_OFF);
        return false;
    }

    return true;
}

bool DepositPuck::execute(RobotState& state)
{
    strat->log("Depositing puck !");
    strat->log((side == LEFT) ? "\tUsing left arm" : "\tUsing right arm");

    float x = MIRROR_X(strat->color, areas[zone_id].pos_x_mm);
    float y = areas[zone_id].pos_y_mm - MIRROR_ARM(side, MIRROR(strat->color, 50));
    float a = MIRROR_A(strat->color, 0);

    if (!strat->goto_xya(strat, x, y, a)) {
        return false;
    }
    strat->gripper_set(side, GRIPPER_RELEASE);
    strat->wait_ms(100);

    strat->gripper_set(side, GRIPPER_OFF);

    pucks_in_area++;
    if (side == LEFT) {
        state.left_has_puck = false;
    } else {
        state.right_has_puck = false;
    }
    state.classified_pucks[areas[zone_id].color]++;
    state.arms_are_deployed = true;
    return true;
}

bool LaunchAccelerator::execute(RobotState& state)
{
    strat->log("Push/launch accelerator !");

    float x = (strat->color == VIOLET) ? 1695 : 1405;

    if (!strat->goto_xya(strat, x, 330, MIRROR_A(strat->color, 90))) {
        return false;
    }

    state.arms_are_deployed = true;
    strat->manipulator_goto(RIGHT, MANIPULATOR_DEPLOY_FULLY);

    strat->forward(strat, -30);
    strat->rotate(strat, MIRROR(strat->color, 20));
    strat->forward(strat, 40);
    strat->manipulator_goto(RIGHT, MANIPULATOR_STORE_FRONT_0);

    state.puck_in_accelerator++;
    state.puck_available[12] = false;
    return true;
}

bool TakeGoldonium::execute(RobotState& state)
{
    strat->log("Taking goldenium !");

    float x = (strat->color == VIOLET) ? 2275 : 825;

    if (!strat->goto_xya(strat, x, 400, MIRROR_A(strat->color, 90))) {
        return false;
    }

    state.arms_are_deployed = true;
    strat->manipulator_goto(RIGHT, MANIPULATOR_PICK_GOLDONIUM);

    if (!strat->goto_xya(strat, x, 300, MIRROR_A(strat->color, 90))) {
        strat->forward(strat, 80);
        return false;
    }

    strat->gripper_set(RIGHT, GRIPPER_ACQUIRE);
    strat->forward(strat, -70);
    strat->wait_ms(500);

    if (!strat->puck_is_picked(RIGHT)) {
        strat->gripper_set(RIGHT, GRIPPER_OFF);
        return false;
    }

    strat->manipulator_goto(RIGHT, MANIPULATOR_LIFT_GOLDONIUM);
    strat->wait_ms(500);
    strat->gripper_set(RIGHT, GRIPPER_OFF);
    strat->forward(strat, 80);

    state.goldonium_in_house = false;
    state.has_goldonium = true;
    return true;
}

bool PutGoldoniumInScale::execute(RobotState& state)
{
    strat->log("Goldenium to the scale !");

    if (!strat->goto_xya(strat, MIRROR_X(strat->color, 1300), 1000, MIRROR_A(strat->color, 270))) {
        return false;
    }
    if (!strat->goto_xya(strat, MIRROR_X(strat->color, 1330), 1359, MIRROR_A(strat->color, 255))) {
        return false;
    }
    if (strat->color == VIOLET) {
        strat->rotate(strat, MIRROR(strat->color, 20));
    }

    state.has_goldonium = false;
    state.arms_are_deployed = true;

    if (!strat->puck_is_picked(RIGHT)) {
        strat->log("I lost the Goldenium on the way !");
        strat->gripper_set(RIGHT, GRIPPER_OFF);
        return false;
    }

    strat->manipulator_goto(RIGHT, MANIPULATOR_SCALE);
    strat->forward(strat, -65);
    strat->gripper_set(RIGHT, GRIPPER_RELEASE);
    strat->wait_ms(200);
    strat->gripper_set(RIGHT, GRIPPER_OFF);
    strat->forward(strat, 40);
    strat->manipulator_goto(RIGHT, MANIPULATOR_STORE_FRONT_2);
    state.arms_are_deployed = false;

    state.puck_in_scale[0] = PuckColor_GOLDENIUM;
    state.num_pucks_in_scale++;
    return true;
}

bool StockPuckInStorage::execute(RobotState& state)
{
    strat->log("Storing puck !");
    strat->log((side == LEFT) ? "\tUsing left arm" : "\tUsing right arm");

    state.arms_are_deployed = true;
    if (storage_id == 0) {
        strat->manipulator_goto(side, MANIPULATOR_STORE_FRONT_HIGH);
    } else if (storage_id == 1) {
        strat->manipulator_goto(side, MANIPULATOR_STORE_FRONT_STORE);
    } else if (storage_id == 2) {
        strat->manipulator_goto(side, MANIPULATOR_STORE_BACK_HIGH);
    } else if (storage_id == 3) {
        strat->manipulator_goto(side, MANIPULATOR_STORE_BACK_STORE);
    } else {
        strat->log("\tStorage in this position is impossible");
        return false;
    }

    if (side == LEFT) {
        state.left_has_puck = false;
    } else {
        state.right_has_puck = false;
    }

    if (!strat->puck_is_picked(side)) {
        strat->log("\tOups, I lost the puck on my way to the storage...");
        strat->gripper_set(side, GRIPPER_OFF);
        return true;
    }

    strat->gripper_set(side, GRIPPER_RELEASE);
    strat->wait_ms(200);
    strat->gripper_set(side, GRIPPER_OFF);

    if (side == LEFT) {
        state.left_storage[storage_id] = state.left_puck_color;
    } else {
        state.right_storage[storage_id] = state.right_puck_color;
    }
    return true;
}

bool PutPuckInScale::execute(RobotState& state)
{
    strat->log("Putting puck in scale !");
    strat->log((side == LEFT) ? "\tUsing left arm" : ((side == RIGHT) ? "\tUsing right arm" : "\tUsing both arms"));

    if (!strat->goto_xya(strat, MIRROR_X(strat->color, 1330), 1359, MIRROR_A(strat->color, 255))) {
        return false;
    }
    if (strat->color == VIOLET) {
        if (side == RIGHT) {
            strat->rotate(strat, MIRROR(strat->color, 20));
        }
    } else {
        if (side == LEFT) {
            strat->rotate(strat, MIRROR(strat->color, 20));
        }
    }
    strat->manipulator_goto(side, MANIPULATOR_SCALE);
    strat->forward(strat, -65);
    strat->gripper_set(side, GRIPPER_RELEASE);
    strat->wait_ms(200);
    strat->gripper_set(side, GRIPPER_OFF);
    state.arms_are_deployed = true;
    strat->forward(strat, 40);
    strat->manipulator_goto(side, MANIPULATOR_STORE_FRONT_2);
    if (side == LEFT) {
        state.left_has_puck = false;
        state.puck_in_scale[state.num_pucks_in_scale] = state.left_puck_color;
    } else {
        state.right_has_puck = false;
        state.puck_in_scale[state.num_pucks_in_scale] = state.right_puck_color;
    }
    state.num_pucks_in_scale++;

    return true;
}

bool PutPuckInAccelerator::execute(RobotState& state)
{
    strat->log("Putting puck in accelerator !");

    float x = 1900 + MIRROR_ARM(side, 50);
    if (!strat->goto_xya(strat, MIRROR_X(strat->color, x), 300, MIRROR_A(strat->color, 90))) {
        return false;
    }
    state.arms_are_deployed = true;
    strat->manipulator_goto(side, MANIPULATOR_PUT_ACCELERATOR);
    if (!strat->goto_xya(strat, MIRROR_X(strat->color, x), 180, MIRROR_A(strat->color, 90))) {
        return false;
    }
    strat->manipulator_goto(side, MANIPULATOR_PUT_ACCELERATOR_DOWN);
    strat->gripper_set(side, GRIPPER_OFF);
    strat->forward(strat, 130);

    if (side == LEFT) {
        state.left_has_puck = false;
    } else {
        state.right_has_puck = false;
    }
    state.puck_in_accelerator++;
    return true;
}

bool PickUpStorage::execute(RobotState& state)
{
    strat->log("Picking up from storage !");

    strat->gripper_set(side, GRIPPER_ACQUIRE);
    if (storage_id == 0) {
        strat->manipulator_goto(side, MANIPULATOR_STORE_FRONT_LOW);
    } else if (storage_id == 1) {
        strat->manipulator_goto(side, MANIPULATOR_STORE_FRONT_HIGH);
    } else if (storage_id == 2) {
        strat->manipulator_goto(side, MANIPULATOR_STORE_BACK_LOW);
    } else if (storage_id == 3) {
        strat->manipulator_goto(side, MANIPULATOR_STORE_BACK_HIGH);
    } else {
        strat->log("\tPicking in this storage position is impossible");
        return false;
    }
    strat->wait_ms(400);

    if (!strat->puck_is_picked(side)) {
        strat->gripper_set(side, GRIPPER_OFF);
        return false;
    }

    if (side == LEFT) {
        state.left_has_puck = true;
        state.left_puck_color = state.left_storage[storage_id];
        state.left_storage[storage_id] = PuckColor_EMPTY;
    } else {
        state.right_has_puck = true;
        state.right_puck_color = state.right_storage[storage_id];
        state.right_storage[storage_id] = PuckColor_EMPTY;
    }
    return true;
}

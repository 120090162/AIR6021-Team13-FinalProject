import time
import mujoco
import mujoco.viewer
from threading import Thread
import threading


locker = threading.Lock()

mj_model = mujoco.MjModel.from_xml_path("model.xml")
mj_data = mujoco.MjData(mj_model)

# 找到 box1 的自由关节 ID
joint_id = mujoco.mj_name2id(mj_model, mujoco.mjtObj.mjOBJ_JOINT, "box1_joint")
if joint_id == -1:
    raise ValueError("未找到关节 'box1_joint'，请确保 XML 中正确命名。")

# 设置 box1 的初始速度（x 方向）
# 自由关节的 qvel：前 3 个是线速度 (x, y, z)，后 3 个是角速度
mj_data.qvel[joint_id * 6 + 0] = 1.0  # x 方向线速度
print(mj_data.qvel)

viewer = mujoco.viewer.launch_passive(mj_model, mj_data)

mj_model.opt.timestep = 0.002

time.sleep(0.2)

# Flag to track if the weld constraint has been activated
welded = False


def SimulationThread():
    global mj_data, mj_model

    while viewer.is_running():
        step_start = time.perf_counter()

        locker.acquire()

    # Check for contacts
    for i in range(mj_data.ncon):
        con = mj_data.contact[i]
        geom1 = con.geom1
        geom2 = con.geom2

        # Get the geometry IDs for box1 and box2
        body_id1 = mujoco.mj_name2id(mj_model, mujoco.mjtObj.mjOBJ_BODY, "box1")
        body_id2 = mujoco.mj_name2id(mj_model, mujoco.mjtObj.mjOBJ_BODY, "box2")
        geom_id1 = mj_model.body_geomadr[body_id1]
        geom_id2 = mj_model.body_geomadr[body_id2]

        # Check if the contact is between box1 and box2
        if (geom1 == geom_id1 and geom2 == geom_id2) or (
            geom1 == geom_id2 and geom2 == geom_id1
        ):
            if not welded:
                # Activate the weld constraint
                weld_id = mujoco.mj_name2id(
                    mj_model, mujoco.mjtObj.mjOBJ_EQUALITY, "weld"
                )
                if weld_id != -1:
                    mj_data.eq_active[weld_id] = 1
                    welded = True
                    print("Weld constraint activated")
                break
        mujoco.mj_step(mj_model, mj_data)

        locker.release()

        time_until_next_step = mj_model.opt.timestep - (
            time.perf_counter() - step_start
        )
        if time_until_next_step > 0:
            time.sleep(time_until_next_step)


def PhysicsViewerThread():
    while viewer.is_running():
        locker.acquire()
        viewer.sync()
        locker.release()
        time.sleep(0.001)


if __name__ == "__main__":
    viewer_thread = Thread(target=PhysicsViewerThread)
    sim_thread = Thread(target=SimulationThread)

    viewer_thread.start()
    sim_thread.start()

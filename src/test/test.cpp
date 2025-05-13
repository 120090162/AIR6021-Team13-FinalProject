#include <mujoco/mujoco.h>
#include <iostream>

int main()
{
    // 加载模型
    mjModel *m = mj_loadXML("model.xml", nullptr, nullptr, 0);
    if (!m)
    {
        std::cerr << "模型加载失败" << std::endl;
        return 1;
    }

    // 创建数据
    mjData *d = mj_makeData(m);
    if (!d)
    {
        std::cerr << "数据创建失败" << std::endl;
        return 1;
    }

    // 设置 box1 的初始速度
    d->qvel[0] = 1.0; // 在 x 方向移动

    bool welded = false;

    while (true)
    {
        mj_step(m, d);

        // 检查接触
        for (int i = 0; i < d->ncon; i++)
        {
            mjContact *con = &d->contact[i];
            int geom1 = con->geom1;
            int geom2 = con->geom2;

            // 获取 box1 和 box2 的几何体 ID
            int body_id1 = mj_name2id(m, mjOBJ_BODY, "box1");
            int body_id2 = mj_name2id(m, mjOBJ_BODY, "box2");
            int geom_id1 = m->body_geomadr[body_id1];
            int geom_id2 = m->body_geomadr[body_id2];

            if ((geom1 == geom_id1 && geom2 == geom_id2) || (geom1 == geom_id2 && geom2 == geom_id1))
            {
                if (!welded)
                {
                    // 激活焊接约束
                    int weld_id = mj_name2id(m, mjOBJ_EQUALITY, "weld");
                    if (weld_id != -1)
                    {
                        d->eq_active[weld_id] = 1;
                        welded = true;
                        std::cout << "焊接约束已激活" << std::endl;
                    }
                    break;
                }
            }
        }

        // 在这里可以添加渲染或其他逻辑
    }

    // 清理
    mj_deleteData(d);
    mj_deleteModel(m);
    return 0;
}
import os

for i in range(40, 45):
    for method in ["cnn", "cnn_single_scene"]:
        for orbit in [True, False]:
            save_dir = "videos/%d_%s" % (i, method)
            if orbit is True:
                save_dir += "_orbit"
            cmd = "python rgbd_tabletop_sim.py --corruption_method=%s " \
                  "--seed=%d --save_dir=%s" % (method, i, save_dir)
            if orbit is True:
                cmd += " --orbit"
            os.system(cmd)
            cmd = "./export_videos.sh %s" % save_dir
            os.system(cmd)
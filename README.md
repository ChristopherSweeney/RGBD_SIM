Pydrake RGBD Simulation
=================

Uses the submitted Keras/CNN-based depth noise learner (see submodule) to produce RGBD simulations of scenes. The critical file for specifying the actual network architecture is `DepthSim/python/RGBDCNN/network.py`, while the pretrained weights are available in `DepthSim/python/models`. `net_depth_seg_v1.hdf5` contains the pretrained weights for the network trained on the entire LabelFusion dataset, while `2017-06-16-30_depth_batch4.hdf5` contains pretrained weights for the network trained on only one scene of the LabelFusion dataset.

Significant reference was made to [this](https://github.com/keras-team/keras/tree/master/docker) Docker reference and [this](https://github.com/gizatt/drake_periscope_tutorial) Pydrake example when writing this. Those are better documented than this repo for matters of Keras, Docker, and [Py]Drake.

## Note to reviewers:
This has been cut down from the full repo for purposes of anonymization (and hopefully easier review). This repo provides a means to reproduce the results running the pretrained CNN against a simulated environment to produce more realistic depth images. We recommend following the "Using a hosted docker image" instructions below.

The simulation runs well below real time due to bottlenecks that we have not engineered around in the Pydrake simulation of complex collision geometries, rendering, and the final noise sampling step (which bottlenecks in generating the Perlin noise map). We feel these are relatively trivial problems that do not detract from the demonstration of the underlying noise prediction mechanism, which is profiled in the paper to run at real time if these other bottlenecks are resolved.

## Gist of how to run this:

### Using a hosted docker image
`make bash` should download the right image and launch a shell script. You can change `IMAGE_NAME` by editing the `Makefile`.

In your host environment, `pip install meshcat` and run `meshcat-server` in a separate terminal. This'll create a host-run visualizer so you can watch the simulation.

Then, `cd workspace` and `python rgbd_tabletop_sim`. Run with `--help` to see options.

You can also run `python run_model_on_labelfusion_scene.py` to run the network directly on reconstructed depth images from our LabelFusion dataset. We included a few images in the folder `2017-06-16-24` -- but only a few frames, due to size restrictions.

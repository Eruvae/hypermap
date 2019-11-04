# Hypermap framework

This framework manages maps with multiple layers. Currently supported are occupancy and semantic layers.

A paper based on this work was submitted to ICRA 2020. You can find the paper [here](https://arxiv.org/abs/1909.09526).

## Related packages

[hypermap_msgs](https://github.com/Eruvae/hypermap_msgs): Messages used by the framework.

[hypermap_rviz_plugin](https://github.com/Eruvae/hypermap_rviz_plugin): Provides a plugin to display semantic layers and hypermaps in rviz.

[semmapping](https://github.com/Eruvae/semmapping): Generate semantic maps from RGB-D camera images.

## Dependencies

This package uses libzip and png++. On Ubuntu, use

```Shell
sudo apt install libzip-dev libpng++-dev
```

to install the required libraries.

## Usage

You can load a saved Hypermap by providing it as argument. Alternatively, ROS parameters can be used (see parameters section).

```Shell
rosrun hypermap map_server map.hmap
```

Save a map from the command-line interface as follows:

```Shell
save map.hmap
```

## Parameters

The following private parameters are available:
- **load_map**: Boolean parameter; load a compressed hypermap file if true.
- **load_map_config**: Boolean parameter; load a hypermap config file if true.
- **file**: Specify the file to load.

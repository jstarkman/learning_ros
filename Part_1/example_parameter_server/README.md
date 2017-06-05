# example_parameter_server

This package illustrates how to read parameters programmatically from a single, global parameter server.

## Example usage

Load parameters from a yaml file via a launch file with:

```
$ launch launch_example_param_server.py
```

Then run the example node with:

```
$ read_param_from_node
```

to demonstrate that the node has obtained the desired parameters from the parameter server.

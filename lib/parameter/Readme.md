# Parameter Service

This module provides functionality to load and update application parameters
in a centralized and thread safe way.

The parameters are organized in a namespace tree hierarchy. There are different
types of parameters:

- Boolean
- Scalar (float32)
- Integer (int32)
- String
- Vector (constant length array of float32)
- Matrix (constant size 2D array of float32)

An efficient polling API is provided to check for parameter changes.

## Configuration Files

Parameters can be loaded from JSON and MessagePack config files.
Parameter namespaces are represented by a dictionary of parameters and
sub-namespaces.
For JSON files comments are allowed using ```#``` to start a comment. This
makes the config file syntax a subset of YAML.

## Example

```c
// create a root-namespace
static parameter_namespace_t root_ns;
parameter_namespace_delcare(&root_ns, NULL, NULL);

// ... somewhere in the control loop initialization code
static parameter_namespace_t pid_ns;
parameter_namespace_delcare(&pid_ns, &root_ns, "pid");
static parameter_t param_kp;
static parameter_t param_ki;
static parameter_t param_kd;
parameter_scalar_declare(&parm_kp, &pid_ns, "kp");
parameter_scalar_declare(&parm_ki, &pid_ns, "ki");
parameter_scalar_declare(&parm_kd, &pid_ns, "kd");

// ... in the control loop:
if (parameter_namespace_contains_changed(&pid_ns)) {
    if (parameter_changed(&param_kp)) {
        pid_set_kp(&pid, parameter_scalar_get(&param_kp));
    }
    if (parameter_changed(&param_ki)) {
        pid_set_ki(&pid, parameter_scalar_get(&param_ki));
    }
    if (parameter_changed(&param_kd)) {
        pid_set_kd(&pid, parameter_scalar_get(&param_kd));
    }
}
ctrl = pid_control(&pid, error);

```

A config file for this example would look like this:
(the syntax is JSON with comments using ```#```, this makes it YAML compatible)

```yaml
{
    "pid": # pid controller gains
    {
        "kp": 1.4,
        "ki": 0.1,
        "kd": 0
    }
}
```
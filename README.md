# Frontier Exploration with a prior

## Dependency

- Python 3.9

### Check your Python version

```sh
python --version
```

---

## Install project as package

```sh
python -m pip install --no-cache git+https://github.com/ekumenlabs/Frontier-Exploration-with-a-prior.git@master
```

---

## Dev workflow

```sh
source dev.sh
```

---


## When you're done:

```sh
deactivate
```


# FESH (Frontier Exploration Scripts Helper)

### After installing the project, you will gain acces to work with the `fesh` commands, which will be described as follows.

**Usage**:

```console
$ fesh [OPTIONS] COMMAND [ARGS]...
```

**Options**:

* `--install-completion`: Install completion for the current shell.
* `--show-completion`: Show completion for the current shell, to copy it or customize the installation.
* `--help`: Show this message and exit.

**Commands**:

* `clean-file`
* `create-random`
* `create-world`

## `clean-file`

**Usage**:

```console
$ clean-file [OPTIONS]
```

**Options**:

* `--file TEXT`: DXF file to clean.  [default: test]
* `--file-dir TEXT`: Workspace directory.  [default: .]
* `--layers TEXT`: Layers from DXF file to work with.  [default: GAZEBO]
* `--object-types TEXT`: Object types from DXF file to work with. (ex. LINE, LWPOLYLINE, etc.)  [default: LINE, LWPOLYLINE]
* `--output-file TEXT`: Cleaned file name.  [default: clean_file.dxf]
* `--help`: Show this message and exit.

## `create-random`

**Usage**:

```console
$ create-random [OPTIONS]
```

**Options**:

* `--file TEXT`: DXF file to use for .world creation.  [default: test]
* `--output-file-dir TEXT`: Directory to save Gazebo files.  [default: /home/.gazebo]
* `--show / --no-show`: Flag for showing Gazebo world after creation.  [default: False]
* `--rectangles INTEGER`: Rectangles used in world creation.
* `--interior-rectangles INTEGER`: Interior rectangles used in world creation.
* `--points INTEGER`: Points used in world creation (rectangles value has higher priority).
* `--x-range INTEGER`: Range in X axis.  [default: 50]
* `--y-range INTEGER`: Range in Y axis.  [default: 50]
* `--wall-thickness FLOAT`: Walls thickness.  [default: 0.1]
* `--wall-height FLOAT`: Walls height.  [default: 5]
* `--cubes INTEGER`: Random cubes to add inside world.
* `--cube-size FLOAT`: Random cubes max size.
* `--help`: Show this message and exit.

## `create-world`

**Usage**:

```console
$ create-world [OPTIONS]
```

**Options**:

* `--file TEXT`: DXF file to use for .world creation.  [default: test]
* `--file-dir TEXT`: Workspace directory.  [default: .]
* `--output-file-dir TEXT`: Directory to save Gazebo files.  [default: /home/.gazebo]
* `--show / --no-show`: Flag for showing Gazebo world after creation.  [default: False]
* `--help`: Show this message and exit.


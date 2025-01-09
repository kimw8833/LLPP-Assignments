# Visualizer 

## Setup venv and install packages

Create a virtual environment.  This will create a virtual environment in .venv.
Activate the venv and install the required packages:

```
$ python3 -m venv .venv
$ source .venv/bin/activate
(.venv) $ pip3 install -r requirements.txt
```

## Enabling the venv after the initial installation

```
$ source .venv/bin/activate
```

## Execute the visualizer

```
(.venv) $ python3 visualize_export.py ../demo/export_trace.bin
```


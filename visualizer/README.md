# Visualizer 

This is a tool to visualize the `Export Trace` that is exported from the
demo executable through the `--export-trace[=export_trace.bin]` flag. If no filename
is provided in the argument, the default filename will be `export_trace.bin`.

To visualize, one needs to setup a python virtual environment and install
necessary packages. If you are working on your own machine go to the next
section. Otherwise, you can jump to the section afterwards (Enabling the
venv...)

## Setup venv and install packages

Create a virtual environment.  This will create a virtual environment in .venv.
Activate the venv and install the required packages:

```
$ python3 -m venv .venv
$ source .venv/bin/activate
(.venv) $ pip3 install -r requirements.txt
```

## Enabling the venv after the initial installation

If you are setting up your virtual environment on your own machine.
```
$ source .venv/bin/activate
```

If you are using the server provided by the teaching staff.
```
$ source /usr/local/share/LLPP_venv/bin/activate
```

You will know that the virtual environment is enabled by the `(.venv)` or
`(LLPP_venv)` at the beginning of your command line.

## Execute the visualizer

Now you can start running the visualizer.
The `visualize_export.py` is the file to run with python. Use the following
command if you want to visualize the trace called `export_trace.bin` in the
demo directory.

Make sure you point to the right files. The `visualize_export.py` python file
is located in the `visualizer` directory. The `export_trace.bin` file (or
whatever you named your export trace output file should be in the directory
that you executed the `demo` executable from.

```
(.venv) $ python3 visualize_export.py ../demo/export_trace.bin
```

Once you execute this a Python GUI window will pop up with the bottoms on the
bottom (Play/stop, next, previous, and a checkbox to enable or disable
heatmaps. Also the slider will enable you to go to a particular step.

# delaunay
## Tele-operation demo
Boris Nikolaevich Delaunay or Delone (Russian: Бори́с Никола́евич Делоне́; March 15, 1890 – July 17, 1980) was one of the first Russian mountain climbers and a Soviet/Russian mathematician. He invented what is now called Delaunay triangulation in 1934.

## Setup

If you intend to use the Sony Sixaxis wireless game controller:
```
sudo apt-get install sixad
```

Start the service before you run any launch files:
```
sixad -start
```

You'll be prompted to input your password. When prompted, press the playstation button the controller to pair.

## Launch

```
roslaunch teleop_support teleop_test.launch mesh:=<ABS_PATH_TO_STL_FILE> [use_joystick:=true]
```

If you use the joystick flag, make sure `sixad` is already running.

## Limitations

There are many, but the first you'll run into is that the system currently only supports '''binary STL''' files that have triangle vertex spacing exceeding ~ 1e-7 m. I would encourage you to 'sanitize' your mesh by removing close vertices using Meshlab.

Known 'good' meshes can be found in the `teleop_test_meshes` directory.

## Bugs

If you find you cant move when you first start, or that the tracking point is moving off the surface. Change to 'Free Mode' then change back.

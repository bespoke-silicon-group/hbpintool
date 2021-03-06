# Building #

`./configure && make`

Note that for the binary-only release you should not run `make` -- just run `configure` to download the relevant pin release.

# Using the Tool #

The tool only supports being run from the source root of `hbpintool`.

From the `hbpintool` directory run these commands to profile your GraphIt program:

1. `source SOURCE_THIS`
2. `./hbpintool /path/to/your/graphit/program [your-graphit-program-arguments]`

You should find the output in a file called `hbpintool.out`

# Compile with -O3

You'll get more accurate measurements if aggresive compiler optimizations are turned on.

# Preventing Compiler Inlining #

For accuracy the tool relies on the GraphIt generated `edgeset_apply` function being defined in the program executable.

However, depending on how the GraphIt C++ output was compiled, this function may have been inlined.

To keep this from happening, open the `.cpp` file generated by `graphitc.py` and add `__attribute__((noinline))` to the definition 
of the templated `edgeset_apply` function (the exact name depends on the GraphIt schedules used).

# Serial Code Only #

This tool is only written to profile serial code (Don't compile the GraphIt program with -fopenmp).

The models it uses to calculate energy factors in the parallelism for the relevant hardware.

# Intel 64 #

This tool can only run on Intel x86_64 processors.

# Evaluating GraphIt programs called by Python3 #

This requires that the GraphIt shared object be compiled e.g. `APPLICATION.cpython-36m-x86_64-linux-gnu.so`.

Invoke `hbpintool` like this:

```
./hbpintool --gtdll /path/to/your/graphit/app.cpython.so `which python3` /path/to/your/python/script.py [your-python-script-arguments]`
```

Note that the `which python3` expands to the absolute path to your Python interpreter and it takes the place of your program in the invocation.

Do **NOT** invoke `hbpintool` on your python script directly. It will not work! 

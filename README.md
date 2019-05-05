# Building #

`./configure && make`

# Using the Tool #

The tool only supports being run from the source root of `hbpintool`.

From the `hbpintool` directory run these commands to profile your GraphIt program:

1. `source SOURCE_THIS`
2. `./hbpintool /path/to/your/graphit/program [your-graphit-program-arguments]`

You should find the output in a file called `hbpintool.out`

# Compile with -O3

You'll get more accurate measurements if turn on compiler optimizations.

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

# Run this file directly from the directory it lives in.

module Example3CCode

using Overdot # Bring in the top-level sim environment.
using Plots # For plotting (Pkg.add("Plots"); Pkg.add("PyPlot");)
import HDF5 # For loading the log file (Pkg.add("HDF5");)
import Juno # For the progress bar
using LinearAlgebra # The scenario file uses functions like 'normalize'.
using Sockets # For ip strings

# Our custom software module has grown since Example 2 and now contains modes
# for running the PD controller as Julia, as C code, or as part of a processor-
# in-the-loop (PITL) test.
include("Example3CustomSoftware.jl")
using .Example3CustomSoftware

# Create the hierarchy of objects used by the sim.
scenario = setup(Scenario(), "basic-cube.yaml", @__MODULE__)

# Set up the controller to run in either:
#
#   jitl: Julia-in-the-loop (same as Example 2)
#   sitl: Software-in-the-loop (compiled C code)
#   pitl: Processor-in-the-loop (requires an external computer on this network running pd-controller-pitl.c)
#
# We can just change the enum we use here to get any of the above modes.
#
scenario.vehicles[1].computers[1].software[1].constants.mode = Example3CustomSoftware.jitl

# Make the log file name show what type of run this is (JITL, SITL, or PITL) so
# that we might compare results from JITL, SITL, and PITL.
scenario.sim.log_file = "out/basic-cube-" * string(scenario.vehicles[1].computers[1].software[1].constants.mode) * ".h5"

# We can set up PITL even when we're not using it. No connections are actually
# created until we run the 'setup' function, and then only if we're in PITL mode.
software_options = scenario.vehicles[1].computers[1].software[1].constants;
software_options.conn.target_ip    = ip"192.168.1.3"
software_options.conn.send_port    = 2000
software_options.conn.receive_port = 2001

# Create a Juno waitbar. This do-block syntax makes sure it gets "closed" even
# if there's an error.
Juno.progress(name="Simulating...") do progress_bar

    # Run the simulation. Anything inside this do-block will be run at the
    # beginning of every major time step. This is useful for updating a progress
    # bar or a plot.
    simulate(scenario) do k, n
        @info "iterating" progress=k/n _id=progress_bar # Update Juno's progress bar.
        return true # Return false to end the sim.
    end

end

# Open the logs and plot some things.
if !isempty(scenario.sim.log_file)

    # Read some data from the logs. This do block takes care of
    # error-handling, so we can just read whatever we want and return it.
    t, q_BI, ω_BI_B =
        HDF5.h5open(scenario.sim.log_file, "r") do logs
            (read(logs, "/cube1/body/state/time"),
             read(logs, "/cube1/body/state/data/q_BI"),
             read(logs, "/cube1/body/state/data/ω_BI_B"))
        end

    # Scalars are logged as 1-by-n (don't know why). Convert to just n for
    # the sake of plotting.
    t = dropdims(t, dims=1)

    # Choose a friendly plotting package. PyPlot behaves nicely.
    pyplot()

    # Define each plot that we'll need.
    p1 = plot(t, transpose(q_BI),
              label  = ["q1" "q2" "q3" "q4"],
              ylabel = "Attitude Quaternion")
    p2 = plot(t, 180/π * transpose(ω_BI_B),
              label  = ["ω1" "ω2" "ω3"],
              ylabel = "Rotation Rate (deg/s)")

    # Save the individual plots before showing them together.
    savefig(p1, "out/sim-results-q.png")
    savefig(p2, "out/sim-results-ω.png")

    # Display those plots as subplots using the default "2" layout. Put any
    # common descriptors here.
    display(plot(p1, p2,
                 layout = 2,
                 xlabel = "Time (s)"))

end # plotting

end # Temp

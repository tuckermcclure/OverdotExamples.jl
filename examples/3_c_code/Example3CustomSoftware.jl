module Example3CustomSoftware

# Export the controller (the only custom software we have at this point). This
# controller expects to find a "gyro" measurement and a "star_tracker"
# measurement in the outputs bus. It expects to sent its outputs (torque
# commands) to an "ideal_actuator".
export StarTrackerAndGyroController

using Overdot

# Bring in the rotations (used by the PD controller).
include(joinpath(dirname(pathof(Overdot)), "OverdotRotations.jl"))
using .OverdotRotations

# Bring in the UDP library (used by the PITL mode).
include(joinpath(@__DIR__, "..", "..", "utilities", "UDPConnections.jl"))
using .UDPConnections

# We'll need this stuff for SITL and PITL.
using Sockets
import Libdl

# We'll pull the PD controller into its own little function here.
function pd_controller(gains, q_TI, q_BI, ω_BI_B)
    theta, r_B = q2aa(qdiff(q_TI, q_BI))
    τ_B        = gains[1] * theta * r_B - gains[2] * ω_BI_B
    f_B        = [0.; 0.; 0.]
    return (f_B, τ_B)
end

# Julia-in-the-loop, software-in-the-loop, and hardware-in-the-loop
@enum StarTrackerAndGyroControllerMode jitl=0 sitl pitl

# The StarTrackerAndGyroController can be run in 3 modes and needs to store that
# mode and the UDP connection.
mutable struct StarTrackerAndGyroControllerConstants
    target::Vector{Float64}
    mode::StarTrackerAndGyroControllerMode
    conn::UDPConnection
    c_lib::Ptr{Cvoid}
    c_fcn::Ptr{Cvoid}
    StarTrackerAndGyroControllerConstants(target = [0.; 0.; 0.; 1.],
                                          mode   = jitl,
                                          conn   = UDPConnection(),
                                          c_lib  = Ptr{Cvoid}(0),
                                          c_fcn  = Ptr{Cvoid}(0)) =
        new(target, mode, conn, c_lib, c_fcn)
end

"""
    StarTrackerAndGyroController
"""
function StarTrackerAndGyroController()

    # Create a set of constants with a (potential) UDP connection. During setup,
    # the target_ip, send_to port, and listen_on ports can be overwritten.
    constants = StarTrackerAndGyroControllerConstants(
                    [0.; 0.; 0.; 1.], # Target quaternion
                    jitl,             # Default operation mode
                    UDPConnection(ip"192.168.1.3", 2000, 2001))

    # This function always runs before the simulation starts up.
    function startup(t, constants, state, args...)
        if constants.mode == sitl
            if Sys.iswindows()
                constants.c_lib = Libdl.dlopen("windows/lib-pd-controller.dll")
            else
                constants.c_lib = Libdl.dlopen("linux/lib-pd-controller.so")
            end
            constants.c_fcn = Libdl.dlsym(constants.c_lib, :pd_controller)
        elseif constants.mode == pitl
            println("Connecting to remote computer.")
            udp_connect(constants.conn)
        end
    end

    # This function is called each time the controller trigger time is reached.
    # It should update the state using its inputs and the outputs from other
    # software or sensors. It should write to its own outputs and to the inputs
    # of any other software or of actuators.
    function step(t, constants, state, inputs, outputs_bus, inputs_bus)

        # Extract what we care about.
        ω_BI_B = outputs_bus["gyro"]
        q_BI   = outputs_bus["star_tracker"]
        q_TI   = constants.target
        gains  = [0.2; 2.]

        # Run the algorithm under development, the C version, or the C version
        # when running on the flight computer.
        if constants.mode == jitl

            f_B, τ_B = pd_controller(gains, q_TI, q_BI, ω_BI_B)

        # Call the C version of the algorithm directly.
        elseif constants.mode == sitl

            # Create array for the outputs. These will be written to by the C
            # function.
            f_B   = [0.; 0.; 0.]
            τ_B   = [0.; 0.; 0.]
            ccall(constants.c_fcn,
                  Nothing,
                  (Ptr{Float64}, Ptr{Float64}, Ptr{Float64}, Ptr{Float64}, Ptr{Float64}, Ptr{Float64}),
                  gains, q_TI, q_BI, ω_BI_B, f_B, τ_B);

        # Send the inputs to the flight computer and wait for its outputs.
        elseif constants.mode == pitl

            # Send over the status and other inputs to the flight software.
            println("Sending...")
            udp_send(constants.conn, 1, gains, q_TI, q_BI, ω_BI_B)

            # Give it just a moment to respond. (Sometimes it gets stuck if it
            # goes to receive too soon.)
            sleep(0.001)

            # Wait for the data to arrive. The inputs here serve as examples for
            # the function, enabling it to know how to write to the outputs.
            println("Receiving...")
            f_B_tuple = (0., 0., 0.)
            τ_B_tuple = (0., 0., 0.)
            f_B_tuple, τ_B_tuple = udp_receive(constants.conn, f_B_tuple, τ_B_tuple)

            # Expand the tuples into vectors.
            f_B = [f_B_tuple...]
            τ_B = [τ_B_tuple...]

        else
            error("Unknown mode.")
        end

        # Write out our command for the ideal actuator.
        inputs_bus["ideal_actuator"] = IdealActuatorCommand(f_B, τ_B)

        return (state, nothing, inputs_bus)

    end

    # Tell the target that we're done, and then close the socket.
    function shutdown(t, constants, state)
        if constants.mode == sitl
            println("Shutting down the StarTrackerAndGyroController library.")
            Libdl.dlclose(constants.c_lib)
        elseif constants.mode == pitl
            println("Shutting down the StarTrackerAndGyroController UDP connection.")
            # Send a status of 0 to indicate that things are over now.
            udp_send(constants.conn, 0)
            udp_close(constants.conn)
        end
    end

    # Create (and, implicitly, return) a DynamicalModel configured to call our
    # functions and use our constants.
    DynamicalModel("star_tracker_and_gyro_controller",
                   startup = startup,
                   init = step,
                   update = step,
                   shutdown = shutdown,
                   timing = ModelTiming(0.05),  # Sample rate
                   constants = constants)

end

end

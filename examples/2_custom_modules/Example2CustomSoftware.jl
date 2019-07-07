module Example2CustomSoftware

# Export the controller (the only custom software we have at this point). This
# controller expects to find a "gyro" measurement and a "star_tracker"
# measurement in the outputs bus. It expects to sent its outputs (torque
# commands) to an "ideal_actuator".
export StarTrackerAndGyroController

using Overdot

# Bring in the rotations (used by the PD controller).
include(joinpath(dirname(pathof(Overdot)), "OverdotRotations.jl"))
using .OverdotRotations

# The StarTrackerAndGyroController has only one constant: the target orientation quaternion.
mutable struct StarTrackerAndGyroControllerConstants
    target::Vector{Float64}
    StarTrackerAndGyroControllerConstants(target = [0.; 0.; 0.; 1.]) = new(target)
end

"""
    StarTrackerAndGyroController
"""
function StarTrackerAndGyroController()

    # Set up the constants used by the controller.
    constants = StarTrackerAndGyroControllerConstants([0.; 0.; 0.; 1.]) # Target quaternion

    # This function is called each time the controller trigger time is reached.
    # It should update the state using its inputs and the outputs from other
    # software or sensors. It should write to its own outputs and to the inputs
    # of any other software or of actuators.
    function step(t, constants, state, inputs, outputs_bus, inputs_bus)

        # Extract what we care about.
        ω_BI_B = outputs_bus["gyro"]
        q_BI   = outputs_bus["star_tracker"]
        q_TI   = constants.target
        gains  = [0.2; 2.] # We'll just hard code the gains for now.

        # Run the algorithm.
        theta, r_B = q2aa(qdiff(q_TI, q_BI))
        τ_B        = gains[1] * theta * r_B - gains[2] * ω_BI_B
        f_B        = [0.; 0.; 0.]

        # Write out our command for the ideal actuator to its inputs.
        inputs_bus["ideal_actuator"] = IdealActuatorCommand(f_B, τ_B)

        # Return the updated stuff.
        return (state, nothing, inputs_bus)

    end

    # Create (and, implicitly, return) a DynamicalModel configured to call our
    # functions and use our constants.
    DynamicalModel("star_tracker_and_gyro_controller",
                   init = step, # During initialization, Overdot can call "step" to see what our outputs look like.
                   update = step, # When updating for a discrete time step, Overdot will call our "step" method.
                   timing = ModelTiming(0.05),  # Set the default sample rate.
                   constants = constants) # Tell Overdot about our constants.

end

end

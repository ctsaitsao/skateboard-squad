# cart_pendulum_template
Cart-pendulum template for Robot Design Studio 2020

## Overview
Application entry point is `main.m`, which calls (through wrapper functions) dynamics-related functions that are automatically generated by running `derive_equations.m`.
That means you have to run `derive_equations.m` before you run `main.m` for the first time.
The workflow is depicted below.

![architecture](/graphics/svg/template_architecture.svg)

`derive_equations.m` uses symbolic computation to generate the state-space dynamics of the cart-pendulum, which are then exported as MATLAB functions (e.g., `autogen_drift_vector_field.m` and `autogen_control_vector_field.m`).
If necessary, symbolic math related to controller design can also be implemented in `derive_equations.m`, but this has not been done yet.

For now, you can do iterative controller design in `main.m`.
Depending on the kinds of controllers you are designing, you might want to create a custom layer of middleware dedicated to controls.


## Attention Students
This code won't run immediately! You have to do your part to make it work.
Specifically, you have to work through `derive_equations.m`.
Anywhere you see the word `TODO`, you have to complete a line of code.

Therefore, this project is a template in the sense that you have to fill in a lot of the math yourself, but thanks to Lagrangian mechanics and (more recently) MATLAB's symbolic math capabilities, that's the easy part.

The hard part is coming up with good software architecture to avoid writing "spaghetti code" (or, at least, to delay code spaghettification as long as possible).
This template provides a good starting point for writing modular and maintainable simulations of rigid-body dynamical systems, but feel free to modify it to suit your needs!

### Autogenerated functions and wrappers
`derive_equations.m` uses `matlabFunction(variable,'File','filename')` extensively, to convert symbolic expressions to MATLAB functions that you can evaluate numerically.
However, these functions often require many input arguments.
For example, the autogenerated function that computes the state-space drift vector field looks like this:

```Matlab
function f_ss = autogen_drift_vector_field(I_pend,b1,b2,dtheta_pend,dx_cart,g,m_cart,m_pend,r_com_pend,theta_pend)
...
```
You don't want to have to supply all those input arguments individually!
Instead, recognize that they can be grouped into two categories:
* robot state `x = [x_cart; theta_pend; dx_cart; dtheta_pend]`
* parameters (literally everything else in the example above)

The solution, then, is to write a wrapper function, `drift_vector_field.m` that parses two input arguments `x` and `params` and calls `autogen_drift_vector_field.m`.
I have included these wrapper functions in this repo so you can see how they work, but you might need to modify them, depending on how you modify `derive_equations.m`.

### MATLAB structs and parsers
In addition to physical parameters (mass, length, etc) that anchor the dynamics in reality, there are many other parameters relevant to simulation, such as timestep size, appearance of the cart-pendulum, etc.
I have grouped _all_ the parameters into a struct called `params`, which is generated by calling `init_params.m`.
This struct gets passed around throughout the simulation, and saves you from having to worry about the order in which you supply input arguments.

Another way to avoid input argument order-dependency (a trademark of brittle code) is to use Name-Value pairs, which you have probably experienced if you've used MATLAB before.
Name-Value pairs rely on an "input parser"; check out `plot_robot.m` and `animate_robot.m` to see how this works.
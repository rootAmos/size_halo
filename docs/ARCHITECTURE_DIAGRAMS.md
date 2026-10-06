# Architecture diagrams

Four diagrams of how the framework is put together. They summarize
[ARCHITECTURE.md](ARCHITECTURE.md), [MODEL_INTERFACES.md](MODEL_INTERFACES.md),
[OPTIMIZATION_PHILOSOPHY.md](OPTIMIZATION_PHILOSOPHY.md) and
[FIDELITY_ROADMAP.md](FIDELITY_ROADMAP.md); those documents remain the
reference. Tier numbers and statuses follow the roadmap table. In every
diagram, solid boxes and edges are implemented, and dashed boxes and edges are
planned. The diagrams read left to right: each column is one group, so they stay
readable on a wide screen instead of scrolling down a single chain.

## 1. Repository organization and layering

Code lives in `src/aircraft_closure/` (the library) and `examples/` (the
problems that are solved). Dependencies point downward only: a lower layer
never imports a higher one. Every layer builds expressions with AeroSandbox
and CasADi; only the orchestration layer creates `asb.Opti` variables,
constraints and objectives. Tests, discipline notebooks, plans and CI sit
beside the code and check it.

```mermaid
flowchart LR
    subgraph L1["1 Orchestration<br/>examples/"]
        direction TB
        sizing["halo_sizing.py<br/>coupled_sizing.py"]
        trajex["trajectory_optimization.py"]
        calib["xv15_reference.py<br/>xv15_performance.py<br/>jvx_rotor_calibration.py"]
    end
    subgraph L2["2 Aircraft assembly<br/>vehicle/"]
        direction TB
        aircraft["Aircraft: wing, tails,<br/>fuselage, items"]
        install["PowertrainInstallation"]
    end
    subgraph L3["3 Disciplines"]
        direction TB
        aero["aerodynamics/"]
        ctrl["controls/"]
        perf["performance/<br/>flight points"]
        mission["mission/<br/>requirements/"]
        traj["trajectory/"]
    end
    subgraph L4["4 Subsystem assemblies"]
        direction TB
        topo["powertrain/topologies.py<br/>series hybrid"]
        coreports["core/: typed ports,<br/>topology, margins"]
    end
    subgraph L5["5 Components<br/>powertrain/components/"]
        direction TB
        comps["Motor, Generator,<br/>Battery, ECM battery,<br/>Turboshaft, Gearbox,<br/>rotors, cables,<br/>heat exchanger"]
    end
    subgraph L6["6 Maps, decks,<br/>empirical data"]
        direction TB
        afdd["weights/afdd.py"]
        decks["powertrain/decks.py"]
        rotordata["data/: JVX tests,<br/>50G cell curves"]
    end
    L1 --> L2 --> L3 --> L4 --> L5 --> L6

    subgraph EXPORT["After the solve<br/>export/openvsp/ (optional)"]
        direction TB
        geom["OpenVSP geometry,<br/>STEP, renders"]
        checks["VSPAERO, parasite drag,<br/>weight back-check"]
        fe["Structure decks,<br/>CalculiX wing check"]
    end
    L1 -. "solved numbers" .-> EXPORT

    ASB[["AeroSandbox and CasADi<br/>asb.Opti, Atmosphere,<br/>MassProperties, AeroBuildup"]]
    L3 -- "builds equations with" --> ASB

    subgraph CHECK["Verification"]
        direction TB
        tests["tests/"]
        notebooks["notebooks/NN_discipline.ipynb"]
        plans["docs/ and .agent/plans/"]
        ci[".github/workflows/"]
    end
    CHECK -. "verifies" .-> L1
```

## 2. Fidelity scaling

Callers (flight points, mission segments, trajectory nodes) talk to each
component through the same small interface: `get_mass()`, `get_limits(...)`
and `evaluate(...)`. When fidelity changes but the operating inputs stay the
same, a submodel is swapped inside the component (turboshaft
`part_power_model` and `lapse_exponent`, machine `mass_model`). When the
inputs change, a distinct class is added: `MomentumProfileRotor` needs rotor
speed, so it sits beside `ActuatorDiskPropulsor` rather than replacing it.
The simplest model is always kept, and named legacy assumption sets (for
example `assumptions_tier12b`) reproduce earlier tiers. Calibration data enter
at the bottom; tests and the discipline notebooks check each step.

```mermaid
flowchart LR
    caller["Callers: flight points,<br/>segments, trajectory nodes"]
    iface["Stable interface<br/>get_mass, get_limits, evaluate"]
    caller -- "same operating inputs" --> iface

    subgraph DATA["Calibration and validation data"]
        direction TB
        jvx["JVX proprotor tests<br/>NASA TM-2016-219070"]
        xv15["XV-15 group weights,<br/>lapse and hover data"]
        gasp["GASP turboshaft deck<br/>user-supplied"]
        cell["Samsung 50G cell data<br/>Paudel et al. 2025"]
    end

    subgraph POWER["Powertrain models"]
        direction TB
        subgraph ROTOR["Rotor: distinct classes"]
            direction LR
            r0["ActuatorDisk<br/>Tier 1"] -- "needs rotor speed" --> r1["MomentumProfile<br/>Tier 12"]
            r1 -.-> r2["BEM rotor<br/>if needed"]:::planned
        end
        subgraph TURB["Turboshaft: submodels"]
            direction LR
            t0["Constant efficiency<br/>Tier 1"] --> t1["Lapse, deck, table<br/>Tiers 10b, 11a"] --> t2["ISA + offset<br/>Tier 16"]
        end
        subgraph MACH["Machines and battery"]
            direction LR
            m0["Specific power<br/>Tier 1"] --> m1["Torque density<br/>Tier 13"]
            b0["Energy capacity<br/>Tier 1"] -- "needs current" --> b1["Equivalent circuit<br/>Tier 17"]
        end
    end

    subgraph AIRFRAME["Airframe models"]
        direction TB
        subgraph AERO["Aerodynamics"]
            direction LR
            a0["Simple<br/>Tier 5"] --> a1["AeroBuildup reference,<br/>Scholz check<br/>Tier 21"]
        end
        subgraph WTS["Weights"]
            direction LR
            w0["Raymer GA, AFDD<br/>Tiers 4, 10a"] --> w1["Tiltrotor wing,<br/>whirl flutter<br/>Tier 20"] --> w2["Cap depth, min gauge,<br/>layout fuselage factor<br/>plans 032, 035"]
        end
        subgraph SYS["Systems layers"]
            direction LR
            e1["Electrical<br/>Tier 15"]
            th1["Thermal<br/>Tier 19"]
            g1["Geometry checks<br/>Tier 23, partial"]
        end
    end

    subgraph VERIFY["Verification"]
        direction TB
        unit["Unit tests: identities,<br/>limits, trends, symbolic"]
        nb["Tier notebooks"]
        legacy["Named legacy sets<br/>reproduce earlier tiers"]
        fecheck["Independent checks:<br/>VSPAERO, CalculiX"]
    end
    iface --> POWER
    iface --> AIRFRAME
    DATA --> POWER
    DATA --> AIRFRAME
    POWER --> VERIFY
    AIRFRAME --> VERIFY

    classDef planned fill:#f6f6f6,stroke:#888888,stroke-dasharray:5 5,color:#555555
```

## 3. One model set, three solve levels

The same component and discipline equations feed three kinds of problem.
Sizing, the mission and the energy allocation are solved together in one
`asb.Opti` on quasi-steady flight points (Tiers 9 to 13). Trajectory
optimization is a separate `asb.Opti` on an already-sized aircraft, with
AeroSandbox point-mass dynamics and direct collocation (Tier 14). A 6-DOF
level is not implemented: it is the planned next fidelity step of the
trajectory layer, using AeroSandbox rigid-body dynamics with the same rotor,
aerodynamic and mass models, extended to moments and inertia. The coupling is
one-way: a sized design goes into the trajectory problem, and its results can
be carried back by hand as better segment definitions or margins; sizing
stays on quasi-steady segments.

```mermaid
flowchart LR
    subgraph MODELS["One set of physical models<br/>src/aircraft_closure/"]
        direction TB
        pt["Powertrain components<br/>rotor, machines, battery,<br/>turboshaft, gearbox"]
        aero["Aerodynamics<br/>AeroBuildup"]
        veh["Vehicle geometry<br/>and MassProperties"]
    end

    subgraph SIZE["Sizing, mission, energy<br/>one asb.Opti, Tiers 9-21"]
        direction TB
        fp["Quasi-steady flight points<br/>requirements as points"]
        seg["Chained mission segments<br/>fuel burn and SOC"]
        clos["Mass closure, stability,<br/>operating margins"]
        fp --> seg --> clos
    end

    subgraph TRAJ["Trajectory optimization<br/>separate asb.Opti, Tier 14"]
        direction TB
        pm["Point-mass dynamics<br/>collocated nodes"]
        tro["Min-energy transition,<br/>min-time climb"]
        pm --> tro
    end

    subgraph AFTER["Checks after the solve<br/>plan 031, numbers only"]
        direction TB
        vsp["OpenVSP geometry"]
        vspaero["VSPAERO and<br/>parasite drag"]
        ccx["CalculiX wing box,<br/>weight back-check"]
        vsp --> vspaero
        vsp --> ccx
    end

    subgraph SIXDOF["6-DOF trajectory<br/>planned"]
        direction TB
        rb["Rigid-body dynamics"]:::planned
        mom["Rotor and aero<br/>moments, inertia"]:::planned
        rb -.-> mom
    end

    MODELS -- "steady points" --> SIZE
    MODELS -- "time nodes" --> TRAJ
    MODELS -. "forces and moments" .-> SIXDOF
    SIZE -- "sized design" --> TRAJ
    SIZE -- "solved numbers" --> AFTER
    AFTER -. "findings become<br/>model options" .-> MODELS
    TRAJ -. "segment definitions" .-> SIZE
    TRAJ -. "attitude states" .-> SIXDOF

    classDef planned fill:#f6f6f6,stroke:#888888,stroke-dasharray:5 5,color:#555555
    style SIXDOF stroke-dasharray:5 5
```

## 4. The coupled sizing problem

`solve_halo_sizing` in `examples/halo_sizing.py` builds one `asb.Opti`. The
models only return expressions; the example adds every constraint and the
objective, and IPOPT solves everything at once. Operating variables (rotor
speed, battery current, generator torque, electric power fraction) are
created per flight point by `build_flight_point`. The hot-day hover at the
destination (Tier 16) is solved in the same problem.

```mermaid
flowchart LR
    subgraph DV["Design variables"]
        direction TB
        dv1["Take-off mass"]
        dv2["Wing position and area,<br/>tail areas, disk area"]
        dv3["Machine torques, speeds,<br/>gear ratio"]
        dv4["Battery power and energy,<br/>fuel, heat exchanger"]
        dv5["Rotor solidity, tip speed"]
    end

    subgraph MV["Mission and<br/>operating variables"]
        direction TB
        mv1["Cruise and loiter speed"]
        mv2["Per point: rotor speed,<br/>battery current, generator<br/>torque, electric fraction"]
    end

    subgraph EQ["Equations from the models"]
        direction TB
        eq1["Aircraft with installed<br/>series-hybrid powertrain"]
        eq2["Mission segments and<br/>requirement flight points"]
        eq3["Mass breakdown and CG"]
    end

    subgraph CON["Constraints"]
        direction LR
        subgraph CONA[" "]
            direction TB
            c1["Mass closure"]
            c2["Requirements: hover,<br/>climb, speed, ceiling"]
            c3["Mission: fuel, reserve,<br/>end SOC"]
            c4["Engine-out hover"]
            c8["Hot-day hover"]
        end
        subgraph CONB[" "]
            direction TB
            c5["Stability: static margin,<br/>Cn_beta, hover trim"]
            c6["Operating margins: speed,<br/>torque, voltage, current"]
            c9["Machine temperatures"]
            c10["Whirl flutter per rev"]
            c7["Rotor clearance, stall"]
        end
    end

    obj(["Objective: minimum<br/>take-off mass or<br/>maximum payload"])
    solve{{"asb.Opti solve<br/>with IPOPT"}}

    DV --> EQ
    MV --> EQ
    EQ --> CON
    CON --> solve
    obj --> solve
```

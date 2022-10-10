---
layout: page
title: Mechanical
permalink: /system/mechanical/
nav_order: 1
parent: System Design
---
# Mechanical
{: .no_toc }

## Table of contents
{: .no_toc .text-delta }
- TOC
{:toc}


<script type="module" src="https://unpkg.com/@google/model-viewer/dist/model-viewer.min.js"></script>
<!-- 3D model viewer style settings -->
<style>
    model-viewer {
        width: 100%;
        height: 40rem;
    }
    .hotspot{
        display: block;
        width: 18px;
        height: 18px;
        border-radius: 9px;
        border: none;
        background-color: #ffc800;
        box-sizing: border-box;
        pointer-events: none;
    }
    .annotation{
        background-color: #FFF;
        position: absolute;
        transform: translate(10px, 10px);
        border-radius: 10px;
        box-shadow: 0 2px 4px rgba(0, 0, 0, 0.25);
        padding: 10px;
        font-size: 12px;
        font-weight: 700;
        max-width: 128px;
        width: max-content;
        height: max-content;
    }
    /* This keeps child nodes hidden while the element loads */
    :not(:defined) > * {
        display: none;
    }
</style>


<model-viewer alt="ESTHER 3D Model" src="../../assets/esther.glb" ar environment-image="" poster="" shadow-intensity="1" auto-rotate camera-controls touch-action="pan-y">
    <button class="hotspot" slot="hotspot-wheelchair" data-position="0.42 0.3 0.3" data-normal="0 0 1">
        <div class="annotation">Wheelchair Base</div>
    </button>
    <button class="hotspot" slot="hotspot-barrett" data-position="0.17 0.85 -0.16" data-normal="0 0 1">
        <div class="annotation">Robot Arm</div>
    </button>
    <button class="hotspot" slot="hotspot-end" data-position="0.2 1.05 -0.09" data-normal="0 0 1">
        <div class="annotation">End Effector</div>
    </button>
</model-viewer>

## Parts

| Item                                | Part Link                                                                                                                                                                                                                                                                  | Comments                              |
| ----------------------------------- | -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- | ------------------------------------- |
| Nema 23 1:10 gearbox (1x)           | [https://www.amazon.com/dp/B097SCZT59/](https://www.amazon.com/dp/B097SCZT59/)                                                                                                                                                                                             |                                       |
| T-Slot rail (82cm)                  | [https://www.mcmaster.com/47065T109/](https://www.mcmaster.com/47065T109/)                                                                                                                                                                                                 |                                       |
| T-Slot corner bracket (4x)          | [https://www.mcmaster.com/3136N172/](https://www.mcmaster.com/3136N172/)                                                                                                                                                                                                   |                                       |
| Motor enclosure (2x)                | [hardware/mechanical/wheelchair/motor_enclosure.stl](https://github.com/CORE-Robotics-Lab/Wheelchair-Tennis-Robot/blob/main/hardware/mechanical/wheelchair/motor_enclosure.stl)                                                                                            | 3D print out of PLA                   |
| Motor enclosure plate (2x)          | [hardware/mechanical/wheelchair/motor_enclosure_plate.DXF](https://github.com/CORE-Robotics-Lab/Wheelchair-Tennis-Robot/blob/main/hardware/mechanical/wheelchair/motor_enclosure_plate.DXF)                                                                                | Water jet out of thin metal           |
| Nema23 to Nema34 adapter plate (2x) | [hardware/mechanical/wheelchair/motor_nema23_nema34_plate.DXF](https://github.com/CORE-Robotics-Lab/Wheelchair-Tennis-Robot/blob/main/hardware/mechanical/wheelchair/motor_nema23_nema34_plate.DXF)                                                                        | Water jet out of thin metal           |
| Clamping shaft coupler (2x)         | [https://www.mcmaster.com/2469K74/](https://www.mcmaster.com/2469K74/)                                                                                                                                                                                                     |                                       |
| Keyed rotary shaft (1x)             | [https://www.mcmaster.com/1439K26/](https://www.mcmaster.com/1439K26/)                                                                                                                                                                                                     | Cut into two 10cm lengths             |
| Machine key (4x)                    | [https://www.mcmaster.com/98870A733/](https://www.mcmaster.com/98870A733/)                                                                                                                                                                                                 |                                       |
| Threaded hex standoffs (8x)         | [https://www.mcmaster.com/94868A767/](https://www.mcmaster.com/94868A767/)                                                                                                                                                                                                 |                                       |
| Motor mount plate left (1x)         | [hardware/mechanical/wheelchair/motor_mount_plate_left.DXF](https://github.com/CORE-Robotics-Lab/Wheelchair-Tennis-Robot/blob/main/hardware/mechanical/wheelchair/motor_mount_plate_left.DXF)                                                                              | Water jet out of metal sheet          |
| Motor mount plate right (1x)        | [hardware/mechanical/wheelchair/motor_mount_plate_right.DXF](https://github.com/CORE-Robotics-Lab/Wheelchair-Tennis-Robot/blob/main/hardware/mechanical/wheelchair/motor_mount_plate_right.DXF)                                                                            | Water jet out of metal sheet          |
| Taper-lock bushing (2x)             | [https://www.mcmaster.com/3753N111/](https://www.mcmaster.com/3753N111/)                                                                                                                                                                                                   |                                       |
| Taper-lock sprocket (2x)            | [https://www.mcmaster.com/57095K421/](https://www.mcmaster.com/57095K421/)                                                                                                                                                                                                 |                                       |
| Axle metal wedge left (1x)          | [https://www.mcmaster.com/6620K78/](https://www.mcmaster.com/6620K78/) [hardware/mechanical/wheelchair/axle_metal_wedge_left.SLDPRT](https://github.com/CORE-Robotics-Lab/Wheelchair-Tennis-Robot/blob/main/hardware/mechanical/wheelchair/axle_metal_wedge_left.SLDPRT)   | Fabricate model out of metal block    |
| Axle metal wedge right (1x)         | [https://www.mcmaster.com/6620K78/](https://www.mcmaster.com/6620K78/) [hardware/mechanical/wheelchair/axle_metal_wedge_right.SLDPRT](https://github.com/CORE-Robotics-Lab/Wheelchair-Tennis-Robot/blob/main/hardware/mechanical/wheelchair/axle_metal_wedge_right.SLDPRT) | Fabricate model out of metal block    |
| Sprocket (2x)                       | [https://www.mcmaster.com/6836N131/](https://www.mcmaster.com/6836N131/) [hardware/mechanical/wheelchair/motor_sprocket_plate.SLDPRT](https://github.com/CORE-Robotics-Lab/Wheelchair-Tennis-Robot/blob/main/hardware/mechanical/wheelchair/motor_sprocket_plate.SLDPRT)   | Drill specified holes out of sprocket |
| Roller chain (2x)                   | [https://www.mcmaster.com/6027K73/](https://www.mcmaster.com/6027K73/)                                                                                                                                                                                                     |                                       |
| Roller chain tensioner (2x)         | [https://www.mcmaster.com/6233K88/](https://www.mcmaster.com/6233K88/)                                                                                                                                                                                                     |                                       |
| Tensioner mount plate (2x)          | [https://www.mcmaster.com/4473T42/](https://www.mcmaster.com/4473T42/)                                                                                                                                                                                                     | Water jet out of metal sheet          |
| Tensioner mount standoff (2x)       | [hardware/mechanical/wheelchair/tensioner_mount_standoff.stl](https://github.com/CORE-Robotics-Lab/Wheelchair-Tennis-Robot/blob/main/hardware/mechanical/wheelchair/tensioner_mount_standoff.stl)                                                                          | 3D print out of PLA                   |

## Drivetrain
The Top End Pro Tennis Wheelchair was used as the base for the drivetrain. This was powered by two ODrive D6374 motors with a 1:20 gear reduction.
### Wheelchair
For the drivetrain mounting framework, attach two 19 cm double t-slot rails to the sides of the frame such that the rails are parallel to the wheels as shown below. These will be the mounting points for the motor and chain drive.
<p style="text-align:center;">
    <img width="50%" src="../../assets/img/mechanical/double_t_slot_to_frame.jpg">
</p>

Attach one 44 cm double t-slot rail perpendicular to the installed 19 cm rails using corner slotted brackets as shown below.
<p style="text-align:center;">
    <img width="50%" src="../../assets/img/mechanical/double_t_slot_cross_piece.jpg">
</p>

### Chain Drive
Assemble the motors with the 3D printed motor enclosures and motor enclosure plates. Attach the motor encoders and NEMA 23 1:10 gearbox to the output shaft of the motor using the NEMA 23 to 34 Adapter plate. This motor and gearbox assembly can be attached to the drivetrain frame using the waterjet drivetrain mounting plates using standoffs and keyed shaft couplers as shown below.
<p style="text-align:center;">
    <img width="80%" src="../../assets/img/mechanical/chain_drive_close.jpg">
</p>

On the output shaft of the gearbox, loosely attach the small sprocket to be tightened later during chain tensioning. Mount the machined large sprockets to the wheel of the drivetrain. Install chain around the wheel and motor sprockets, and tighten the motor sprocket on so that the sprockets and chain are aligned. Finally, attach the tensioner and slide downwards onto the chain to remove slack. The final assembly can be seen below.
<p style="text-align:center;">
    <img width="80%" src="../../assets/img/mechanical/chain_drive_side.jpg">
</p>

## Manipulator
A HEAD Graphene Instinct Power tennis racket was attached to a 7-degree-of-freedom Barrett robot arm was 
### Barrett Arm
To attach the robot arm to the wheelchair, remove the cloth seat, handles, and backrest from the wheelchair. The steel waterjet seat plate can then be attached to the wheelchair and used as an anchor point for the robot arm.

### End Effector
Attach the tennis racket to the end of the robot arm using two bolted 3D printed racket holders which sandwich the end of the manipulator as shown below.
<p style="text-align:center;">
    <img width="50%" src="../../assets/img/mechanical/racket_holder.png">
</p>

Since the racket holder is an area of high stress during a swing, a visualization of the possible failure points within the part was created. This can be seen below where red areas indicate high stress. Based on this analysis and initial testing, zip ties were used to secure and stabilize the racket to the racket holder.
<p style="text-align:center;">
    <img width="49%" src="../../assets/img/racket_clamp_stress.png">
    <img width="49%" src="../../assets/img/racket_clamp_stress_animation.gif">
</p>


---

{: .highlight }
Now check out the [electrical](https://core-robotics-lab.github.io/Wheelchair-Tennis-Robot/system/electrical/) page to see how to setup the electrical system. 

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
Hello my name is

{: .highlight }
A paragraph

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

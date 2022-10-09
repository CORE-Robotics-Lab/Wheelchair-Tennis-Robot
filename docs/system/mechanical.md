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


<model-viewer alt="ESTHER 3D Model" src="../../assets/esther_.glb" ar environment-image="" poster="" shadow-intensity="1" auto-rotate camera-controls touch-action="pan-y">
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


## Drivetrain
<img width="100%" src="../../assets/img/paper/chain_drive_2.png">
<img width="100%" src="../../assets/img/paper/chain_drive_3.png">

### Wheelchair
### Chain Drive

## Manipulator
### Barrett Arm
### End Effector
<img width="100%" src="../../assets/img/racket_clamp_stress.png">
<img width="100%" src="../../assets/img/racket_clamp_stress_animation.gif">

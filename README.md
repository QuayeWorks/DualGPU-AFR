# DualGPU-AFR (Experimental)

A DirectX 11 interception-based runtime exploring user-controlled multi-GPU rendering without driver or engine support.

## Overview

Implicit multi-GPU technologies such as SLI and CrossFire are effectively deprecated.

Explicit multi-GPU in DirectX 12 and Vulkan requires engine-level implementation and is rarely used in practice.

This project explores an alternative approach:

Application-layer multi-GPU control

No driver profiles. No engine modification.

## What This Project Does

- Hooks DirectX 11 at runtime (`dxgi.dll`, `d3d11.dll`)
- Observes frame presentation and GPU resource behavior
- Builds toward Alternate Frame Rendering (AFR) outside driver control
- Enables dual-GPU experimentation in real applications

## Current Status

Implemented:

- Multi-adapter enumeration and D3D11 device creation
- Dual-GPU compute and rendering validation
- GPU to GPU transfer (CPU path validated)
- AFR simulation with near-linear scaling in controlled environments
- DXGI proxy injection into real DirectX 11 applications
- D3D11 proxy with resource telemetry
- Real-world application observation

In progress:

- Resource classification
- Resource mirroring feasibility
- Frame pacing validation in real applications

## Architecture

Game  
↓  
dxgi.dll (proxy)  
↓  
d3d11.dll (proxy)  
↓  
Real DirectX runtime  
↓  
GPU(s)  

DXGI proxy:
- Intercepts swap-chain creation
- Tracks Present calls
- Monitors frame timing

D3D11 proxy:
- Hooks device and device context
- Logs resource creation and updates
- Provides the foundation for resource mirroring

## Goals

- Demonstrate AFR outside driver control
- Understand real-world engine resource behavior
- Build a reusable multi-GPU runtime
- Evaluate feasibility across DirectX 11 titles

## Non-Goals (Current Scope)

- DirectX 12 or Vulkan support
- Anti-cheat compatibility
- Universal game support
- Production-ready stability

## Usage (Experimental)

1. Build the project
2. Copy the following into the target application directory:

   dxgi.dll  
   d3d11.dll  

3. Launch the application normally

Logs are generated alongside the executable.

## Roadmap

Phase 7: Resource telemetry and classification  
Phase 8: Controlled resource mirroring  
Phase 9: First real-world AFR attempt  
Phase 10: Stability and compatibility improvements  

## Early Findings

- Near-linear scaling is achievable in controlled AFR scenarios
- Frame scheduling is manageable at the application layer
- Resource synchronization is the primary challenge in real engines

## Future API Support

Planned:

- DirectX 9 via d3d9.dll proxy
- DirectX 10 via DXGI interception
- Evaluation of DirectX 12 and Vulkan

Each API requires a separate backend due to differences in resource handling.

## Disclaimer

This project is experimental and intended for research purposes.

It may cause crashes or undefined behavior and may not work across all systems or applications.

## License

To be added

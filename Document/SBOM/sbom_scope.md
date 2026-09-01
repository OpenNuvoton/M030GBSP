# Introduction

The M030G MCU BSP SBOM package consists of two CycloneDX SBOM files: a Product SBOM and a Test Sample SBOM. The Product SBOM includes components that are included in, linked with, deployed to, or reasonably expected to be integrated into the final product firmware or software. The Test Sample SBOM includes components used only for samples, demonstrations, validation, or testing and not included in the product runtime unless explicitly integrated by the user.

# Product SBOM

Used for CRA compliance, product vulnerability management, software composition analysis, and customer product integration risk assessment.

## Scope

Drivers, device support files, startup code, linker configuration, libraries, binaries, and source components that are compiled, linked, flashed, deployed, or reasonably expected to be integrated into product firmware by customers.

## Evidence

.<br>
└── Library<br>
&nbsp;&nbsp;&nbsp;&nbsp;├── CMSIS<br>
&nbsp;&nbsp;&nbsp;&nbsp;├── Device<br>
&nbsp;&nbsp;&nbsp;&nbsp;└── StdDriver

## Manual SCA Evidence

Arm CMSIS under `Library/CMSIS` is a third-party Product component. The delivered CMSIS package version is 6.1.0 and its license is Apache-2.0. Manual evidence is stored under `Document/SBOM/components/ProductView`.

## Excluded GCC Semihosting Support Files

The repository contains the following GCC semihosting support files:

- `Library/Device/Nuvoton/M030G/Source/GCC/_syscalls.c`
- `Library/Device/Nuvoton/M030G/Source/GCC/semihosting.h`

The files contain content originating from the micro-os-plus
distribution, and `_syscalls.c` also contains newlib-derived portions.

No current M030G project, build script, linked source resource, or
retained build artifact was found to compile or link these files.
Therefore, the files are excluded from the Product SBOM pending
documented build evidence showing product firmware integration.

The source files remain in the repository. The corresponding review
evidence is retained under:

`Document/SBOM/components/Excluded/`

# Test Sample SBOM

Used for transparent disclosure and internal or customer evaluation, but labeled as a non-product runtime dependency.

## Scope

Sample code, demonstration projects, validation code, project configuration, sample firmware images, and test-related content delivered under `SampleCode`.

## Evidence

.<br>
└── SampleCode<br>
&nbsp;&nbsp;&nbsp;&nbsp;├── Hard_Fault_Sample<br>
&nbsp;&nbsp;&nbsp;&nbsp;├── ISP<br>
&nbsp;&nbsp;&nbsp;&nbsp;├── Semihost<br>
&nbsp;&nbsp;&nbsp;&nbsp;├── StdDriver<br>
&nbsp;&nbsp;&nbsp;&nbsp;└── Template

## FMC IAP Binary Artifacts

The FMC IAP sample contains four toolchain-specific `.bin` firmware images under `SampleCode/StdDriver/FMC_IAP`. These files are referenced by the corresponding sample projects and are Test Sample build artifacts. They are not Product runtime components. Each file is represented as a CycloneDX file component with its exact repository-relative path and SHA-256 hash.

# Excluded Repository Content

The `Document` directory, root `.vscode` directory, `Readme.pdf`, and root `vcpkg-configuration.json` are not software component scopes. `Document/SBOM` contains governance records and manual SCA evidence and must not be treated as Product or Test Sample software components.

# M030G-Specific Scope Constraints

M030G is not suitable for FreeRTOS integration in this BSP scope. The M030G BSP does not use and is not expected to include top-level `ThirdParty` or `Tool` directories. If either directory appears in a future release, the SBOM scope must be reviewed before release evidence is generated.

# External Build Dependencies

GCC project files may select newlib-nano through toolchain options such as `--specs=nano.specs`. This is an external build-toolchain dependency and does not mean that the complete newlib or newlib-nano source package is distributed in the M030G BSP repository.

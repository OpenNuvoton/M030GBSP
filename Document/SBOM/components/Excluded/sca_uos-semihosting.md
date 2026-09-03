# uOS++ III Semihosting and Syscall Support Component Description

This document records manual SCA evidence for GCC semihosting and syscall support files delivered in the M030G BSP.

## 1) Component Identity

- Component name (`name`): `uOS++ III Semihosting and Syscall Support`
- Component type (`type`): `library`
- Author: `Liviu Ionescu`
- Version: `NOASSERTION`
- Origin: Third-party or mixed-origin
- License conclusion: `NOASSERTION`
- Review required: Yes

## 2) Evidence Files

- `Library/Device/Nuvoton/M030G/Source/GCC/_syscalls.c`
  - SHA-256: `bc01f2e8a9b9d568fa40c74d75194c919eaf4859da7cbf352a2a3005c396719f`
  - File length: `25007` bytes
- `Library/Device/Nuvoton/M030G/Source/GCC/semihosting.h`
  - SHA-256: `1f28eb5ea069c0bcade5ef4240aaa28687788c3747e5bf226cd37814955420b4`
  - File length: `3759` bytes

## 3) Source and Provenance Evidence

- Both files identify themselves as part of the uOS++ III distribution.
- Both files identify Liviu Ionescu as the copyright holder.
- The exact upstream release, tag, and commit are not identified in the delivered files.
- The M030G files are not byte-for-byte identical to the corresponding M55M1 files.
- M55M1 does not contain existing manual SCA evidence for these files under `Document/SBOM`.

## 4) License Analysis

### semihosting.h

- The file identifies itself as part of the uOS++ III distribution.
- The delivered file does not include a complete license grant or SPDX identifier.
- MIT is a candidate upstream license, but the exact source revision has not been verified.
- The license conclusion remains `NOASSERTION` pending exact-source verification.

### _syscalls.c

- The file identifies itself as part of the uOS++ III distribution.
- The file states that portions originate from newlib sources and were issued under GPL.
- The file does not identify the GPL version, only-or-later semantics, an exception, or the exact newlib revision.
- No specific GPL SPDX expression is asserted.
- The license conclusion remains `NOASSERTION` pending formal license review.

## 5) CycloneDX Modeling Guidance

- Represent the delivered files as one manual mixed-origin component for inventory and review tracking.
- Use component version `NOASSERTION` because the exact upstream revision is not identified.
- Do not assign `Apache-2.0` based only on the parent Nuvoton Device directory.
- Do not convert the incomplete GPL wording into a specific SPDX expression.
- Preserve the actual SHA-256 hash of each evidence file in component properties.
- Do not generate a synthetic component content hash from component metadata.
- Do not add the component as a root dependency solely to satisfy an SBOM checker.

## 6) Compliance Notes

- Review status: `required`
- License status: `unresolved`
- SPDX coverage exception applies to the two evidence files listed above.
- The files must not be modified merely to achieve 100 percent SPDX header coverage.
- External newlib-nano selected through GCC project options is a build-toolchain dependency and is not treated as bundled newlib source.
- Formal license approval is required before replacing `NOASSERTION` with a specific license expression.
## 7) Canonical Component Content Hash

- Algorithm: `sha256-path-nul-content-nul-v1`
- File count: `2`
- SHA-256: `8cc2ebf35c43b6d1e343919f80be2dc7b5d85cf273224fb4d3d720bd2a1dd2e3`

The hash covers `_syscalls.c` and `semihosting.h`. Files are ordered by repository-relative path. For each file, the SHA-256 input contains the UTF-8 relative path, a NUL byte, the raw file bytes, and a terminating NUL byte.

This aggregate hash provides component content integrity only. It does not identify an upstream package, release, tag, commit, supplier, PURL, CPE, or license. The license conclusion remains `NOASSERTION`, and formal license review remains required.
## 8) SBOM Compliance Representation

The CycloneDX component uses the following unversioned generic package
identifier because no exact upstream release, tag, or commit has been
identified:

`pkg:generic/micro-os-plus/semihosting-and-syscall-support`

The unresolved license review state is represented using the following
CycloneDX license expression:

`LicenseRef-uOS-III-License-Review-Pending`

This LicenseRef is a review-state representation. It does not assert that
the complete component is covered by a single standard SPDX license.

The component version remains `NOASSERTION`. The license status remains
`unresolved`, and formal review of the uOS++ III and newlib-derived source
content remains required before final license approval.
## SBOM Scope Disposition

- Disposition: Excluded from Product SBOM
- Review date: 2026-08-29
- Files reviewed:
  - Library/Device/Nuvoton/M030G/Source/GCC/_syscalls.c
  - Library/Device/Nuvoton/M030G/Source/GCC/semihosting.h
- Direct project or build reference found: No
- Linked source reference found: No
- Retained build artifact found: No
- Product firmware integration evidence found: No

The M030G repository contains GCC semihosting support files originating
from the micro-os-plus distribution. The _syscalls.c file also contains
newlib-derived content.

No current M030G project, build script, linked source resource, or retained
build artifact was found to compile or link these files. The files are
therefore excluded from the Product SBOM pending documented build evidence
showing product firmware integration.

This disposition does not remove or modify the source files. The evidence
is retained for traceability and future license review.

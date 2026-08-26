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
- Use component version `NOASSERTION` because the exact upstream revision is unknown.
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

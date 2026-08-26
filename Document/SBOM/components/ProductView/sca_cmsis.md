# CMSIS Third-Party Component Description (for SCA / SBOM)

This document provides SBOM-ready metadata for the vendored Arm CMSIS source package under `Library/CMSIS`.

## 1) Component Identity

- Component name (`name`): `CMSIS`
- Component type (`type`): `library`
- Supplier / project: `Arm Limited`
- Release version: `6.1.0`
- License: `Apache-2.0` (SPDX)
- Evidence path: `Library/CMSIS`
- Upstream repository: `https://github.com/ARM-software/CMSIS_6`
- Upstream tag: Not asserted
- Upstream commit: Not asserted

## 2) Included Content

This component covers the delivered CMSIS content under the following paths:

- `Library/CMSIS/Core`
- `Library/CMSIS/CoreValidation`
- `Library/CMSIS/Driver`
- `Library/CMSIS/RTOS2`
- `Library/CMSIS/Documentation`

The delivered Core directory also contains templates and test sources. These files are part of the delivered CMSIS source package, but their inclusion does not imply that every file is linked into the final M030G product firmware.

## 3) Evidence for Version and License

Primary evidence in this repository:

- `Library/CMSIS/Core/Include/cmsis_version.h`
  - Header includes `SPDX-License-Identifier: Apache-2.0`
  - `__CM_CMSIS_VERSION_MAIN` is `6U`
  - `__CM_CMSIS_VERSION_SUB` is `1U`
  - `__CA_CMSIS_VERSION_MAIN` is `6U`
  - `__CA_CMSIS_VERSION_SUB` is `1U`
- `Library/CMSIS/Documentation/html/General/footer.js`
  - Documentation identifies the CMSIS package version as `6.1.0`
- `Library/CMSIS/Documentation/html/Driver/footer.js`
  - Documentation identifies the CMSIS-Driver version as `2.10.0`
- `Library/CMSIS/RTOS2/Include/cmsis_os2.h`
  - Header includes `SPDX-License-Identifier: Apache-2.0`
  - Header identifies the CMSIS-RTOS2 API version as `2.3.0`
- `Library/CMSIS/Driver/Include/Driver_Common.h`
  - Header includes `SPDX-License-Identifier: Apache-2.0`
  - Header identifies the common driver definitions API version as `2.0`

## 4) License Handling Guidance

For CycloneDX output, use the SPDX license identifier directly:

- `licenses[0].license.id = Apache-2.0`

Preserve the upstream copyright and license headers in all vendored source files. Do not apply the Nuvoton source license as a replacement for the Arm CMSIS license evidence.

## 5) Suggested CycloneDX Field Mapping

- `bom-ref`: `pkg:github/ARM-software/CMSIS_6@6.1.0?source=vendored&path=Library/CMSIS`
- `purl`: `pkg:github/ARM-software/CMSIS_6@6.1.0`
- `licenses[0].license.id`: `Apache-2.0`
- `scope`: `required`
- `bsp:component-origin`: `third-party`
- `bsp:evidence-file`: `Document/SBOM/components/ProductView/sca_cmsis.json`
- `bsp:evidence-path`: `Library/CMSIS`

## 6) Compliance Notes

- Keep all original Arm copyright and license notices.
- The exact upstream tag and commit are not asserted because a byte-level comparison with a specific upstream revision has not been completed.
- Do not copy the M55M1 CMSIS version, upstream tag, upstream commit, or component hashes into the M030G evidence.
- If finer-grained component tracking is required later, CMSIS-Core, CMSIS-Driver, CMSIS-RTOS2, and CoreValidation may be reviewed as separate components.
- Generated documentation content is treated as part of the delivered CMSIS package and must not be interpreted as proof that every documented CMSIS module is present as source code.
## 7) Canonical Component Content Hash

- Algorithm: `sha256-path-nul-content-nul-v1`
- File count: `2253`
- SHA-256: `b5c3e4f30f88852f4433bfd818a8f091654b8b5ec099b4a5b868c0ecb79c46ad`

The hash is computed from all files under `Library/CMSIS`. Files are ordered by repository-relative path. For each file, the SHA-256 input contains the UTF-8 relative path, a NUL byte, the raw file bytes, and a terminating NUL byte. This is an evidence-backed content hash and is not derived from component metadata.

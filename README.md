# M030GBSP

See `Readme.pdf` for BSP usage and release information.

## SBOM

The repository contains separate CycloneDX 1.6 Product and Test Sample SBOMs:

- `M030GBSP_Product_SBOM_cdx.json`
- `M030GBSP_TestSample_SBOM_cdx.json`
- `M030GBSP_SBOM_Manifest.json`

Scope policies and manual component evidence are maintained under
`Document/SBOM`. The Product SBOM covers the runtime library scope. The Test
Sample SBOM covers `SampleCode/` and records every tracked `.a`, `.bin`, `.dll`,
`.exe`, and `.lib` artifact with its exact repository-relative path and
SHA-256 hash.

The formal release package for the current source commit is stored in the SBOM
release repository under `bsp/m030g/V3.04.000-20-gd9fc2b3e`. It includes
strict checker reports, vulnerability scan metadata and limitations,
release-validation configuration, and complete SHA-256 payload checksums.

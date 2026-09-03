# Device License Review Evidence

- Baseline Git commit: `f5d1bb9d41f925d9bfb521c41fa1331689da7b01`
- Release status: blocked pending authoritative legal conclusions.
- Authority search result: no SPDX identifier, copyright notice, license grant,
  exact-hash license mapping, or path-specific license evidence was found for
  the four linker-control files in the repository, Git history, current SBOM
  evidence, or the read-only SVN SBOM package.
- The SVN r51 authority at
  `http://nthcrdvss01.nuvoton.com:9443/svn/SBOM/bsp/m030g/V3.04.000-27-gfcbbddf8/Nuvoton%20Software%20License%20Agreement.md@51`
  has SHA-256
  `8c61cd7eb3a4bd8693bb3678c57c9568d328ad661dc5f4682fcf306644dcf9fe`.
  It authorizes distribution in binary form only and therefore cannot establish
  source redistribution rights for the `.sct` and `.ld` files. The agreement
  text is not copied into Git because its Git redistribution permission has not
  been established.
- The four linker-control files are MS00-006 blockers because replacing the
  overlapping `Library/Device` aggregate requires per-file license conclusions.
  The two excluded semihosting files remain separate MS00-006 legal blockers.

## Library/Device/Nuvoton/M030G/Source/ARM/APROM.sct
- SHA-256: `7c34d7a8dbe95a8bd60b4b0611fb81512db8cdb7655feb765dd494f706023ea8`
- Git blob SHA: `424473f066336de5c07caabe8700d52713a35278`
- Status: `legal-review-pending`
- Required decision: Legal must establish an authoritative license conclusion and redistribution obligations for this exact file before release.

## Library/Device/Nuvoton/M030G/Source/ARM/LDROM.sct
- SHA-256: `f1c421848135a2ab04e61e49d83269bc090564f33e8100396d62e8945c47ece7`
- Git blob SHA: `ee83d03276cfbc4bc6c802e601ad2f527241dcac`
- Status: `legal-review-pending`
- Required decision: Legal must establish an authoritative license conclusion and redistribution obligations for this exact file before release.

## Library/Device/Nuvoton/M030G/Source/GCC/gcc_arm.ld
- SHA-256: `e3651fbd1c4effc6e5f36090d620a51bc7e7db2e0e457a4a929f5490e385c1bf`
- Git blob SHA: `c89c66650e8aa5c677154edc17b9673a2cb0e60d`
- Status: `legal-review-pending`
- Required decision: Legal must establish an authoritative license conclusion and redistribution obligations for this exact file before release.

## Library/Device/Nuvoton/M030G/Source/GCC/LDROM.ld
- SHA-256: `1ee51de0ba2762f19e6ea32213f3791cda4618f22789f23f03c85f3a2bc9bb28`
- Git blob SHA: `9250f559120b9bcc7f1f3b216a495e5c6391c91b`
- Status: `legal-review-pending`
- Required decision: Legal must establish an authoritative license conclusion and redistribution obligations for this exact file before release.

## Library/Device/Nuvoton/M030G/Source/GCC/_syscalls.c
- SHA-256: `bc01f2e8a9b9d568fa40c74d75194c919eaf4859da7cbf352a2a3005c396719f`
- Git blob SHA: `63b2034dad7916e240b0b9cb8c7aa696db804fb2`
- Status: `legal-review-pending`
- Required decision: Legal must establish an authoritative license conclusion and redistribution obligations for this exact file before release.

## Library/Device/Nuvoton/M030G/Source/GCC/semihosting.h
- SHA-256: `1f28eb5ea069c0bcade5ef4240aaa28687788c3747e5bf226cd37814955420b4`
- Git blob SHA: `0c60551b5acf35c1baf6bc758dd73b614d610219`
- Status: `legal-review-pending`
- Required decision: Legal must establish an authoritative license conclusion and redistribution obligations for this exact file before release.

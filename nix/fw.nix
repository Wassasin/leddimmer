{ stdenv, esp-idf, shortRev }:

stdenv.mkDerivation (o: {
  pname = "leddimmer-fw";
  version = "0.1.0-${shortRev}";
  src = ../fw;

  nativeBuildInputs = [ esp-idf ];
  IDF_PATH = "${esp-idf}";
  IDF_PYTHON_ENV_PATH = "${esp-idf.pyEnv}";
  IDF_TOOLS_PATH = "${esp-idf}/tool";
  IDF_PYTHON_CHECK_CONSTRAINTS = 0;
  IDF_TARGET = "esp32c3";
  SDKCONFIG_DEFAULTS = "${../fw/sdkconfig.default}";
  phases = "unpackPhase buildPhase installPhase fixupPhase";

  dontStrip = true;
  dontFixup = true;
  dontPatchELF = true;

  buildPhase = ''
    echo '${o.version}' > version.txt
    source ${esp-idf}/export.sh
    idf.py build
  '';
  installPhase = ''
    cd build
    mkdir -p $out/partition_table $out/bootloader
    cp partition_table/partition-table.bin $out/partition_table
    cp bootloader/bootloader.bin $out/bootloader
    cp leddimmer-fw.{bin,elf} flasher_args.json $out/
  '';

  # lose the context from esp-idf
  # requires structured attrs + nix 2.18
  # unsafeDiscardReferences.out = true;

  shellHook = ''
    source $IDF_PATH/export.sh
  '';
})

{ stdenv, stdenvNoCC, esp-idf, fetchgit, fetchurl, python2, python3
, autoPatchelfHook, libusb1, zlib, git, perl, gdbgui, lib, sources, cmake
, platform ? "linux-amd64" }:

let
  src = sources.esp-idf;

  tools =
    (builtins.fromJSON (builtins.readFile "${src}/tools/tools.json")).tools;

  toDownload = tool:
    let
      version = lib.findFirst ({ status, ... }: status == "recommended") null
        tool.versions;
      download = version."${platform}" or null;
    in if download != null then
      fetchurl { inherit (download) url sha256; }
    else
      null;

  toolsTars = (builtins.filter (x: x != null) (builtins.map toDownload tools));

  # Based on requirements.txt of esp-idf
  pythonIdf = python3.withPackages (p:
    [ gdbgui ] ++ (with p; [
      bitstring
      click
      construct
      cryptography
      ecdsa
      future
      idf-component-manager
      kconfiglib
      pyelftools
      pyparsing_2_3
      pyserial
      reedsolo
      setuptools
    ]));

  # Used by xtensa-esp32-elf-gdb
  python2Insecure = python2.overrideAttrs (o: { meta.insecure = false; });

in stdenv.mkDerivation {
  pname = "esp-idf";
  version = "4.4.3";
  inherit src;
  buildInputs = [ pythonIdf stdenv.cc.cc.lib python2Insecure libusb1 zlib ];
  nativeBuildInputs = [ autoPatchelfHook ];
  passthru.python = pythonIdf;
  propagatedBuildInputs = [ pythonIdf git perl ];

  buildPhase = ''
    sed \
      -e '/^gdbgui/d' \
      -e '/^kconfiglib/c kconfiglib' \
      -e '/^construct/c construct' \
      -i requirements.txt
    echo v$version > version.txt
    export IDF_TOOLS_PATH=$out/tool
    mkdir -p $out/tool/dist
    ${ # symlink all tools
      lib.concatMapStringsSep "\n" (tool: ''
        ln -s ${tool} $out/tool/dist/$(echo ${tool} | cut -d'-' -f 2-)
      '') toolsTars
    } 
    patchShebangs .
    python3 ./tools/idf_tools.py install
    # remove .tar.gz dependencies
    rm -rf $out/tool/dist
  '';

  installPhase = ''
    cp -r ./. $out/
    mkdir $out/bin
    ln -s $out/tool/tools/*/*/*/bin/* $out/bin/
  '';

  passthru.tests.runs = stdenvNoCC.mkDerivation rec {
    name = "idf.py-runs";
    buildInputs = [ cmake esp-idf ];
    phases = "buildPhase";
    buildPhase = ''
      export HOME=/tmp
      source ${esp-idf}/export.sh
      idf.py --version
      touch $out
    '';
  };
}

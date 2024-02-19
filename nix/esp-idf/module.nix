{ config, lib, dream2nix, packageSets, ... }:
let
  src = config.deps.esp-idf;
  platform = "linux-amd64";

  tools =
    (builtins.fromJSON (builtins.readFile "${src}/tools/tools.json")).tools;

  toDownload = tool:
    let
      version = lib.findFirst ({ status, ... }: status == "recommended") null
        tool.versions;
      download = version."${platform}" or version.any or null;
    in if download != null then
      config.deps.fetchurl { inherit (download) url sha256; }
    else
      null;

  toolsTars = (builtins.filter (x: x != null) (builtins.map toDownload tools));

in {
  imports = [ dream2nix.modules.dream2nix.pip ];

  name = "esp-idf";

  mkDerivation = {
    src = config.deps.esp-idf;
    buildInputs = with config.deps; [ libstdcxx libusb1 zlib ];
    nativeBuildInputs = [ config.deps.autoPatchelfHook ];
    propagatedBuildInputs = with config.deps; [ git perl cmake ];
    buildPhase = ''
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
      rm $out/tool/tools/*/*/*-gdb/bin/*-gdb-3.{6,7,8,9,11}
      ln -fs $out/tool/tools/*/*/*/bin/* $out/bin/
    '';
  };

  buildPythonPackage.format = "other";

  deps = { nixpkgs, ... }: {
    libstdcxx = nixpkgs.stdenv.cc.cc.lib;
    inherit (nixpkgs) libusb1 zlib autoPatchelfHook git perl fetchurl cmake;
  };
  env.dontUseCmakeConfigure = true;

  pip = {
    pypiSnapshotDate = "2023-11-18";
    # TODO: merge with
    # 'https://dl.espressif.com/dl/esp-idf/espidf.constraints.v5.1.txt'
    # workaround: IDF_PYTHON_CHECK_CONSTRAINTS=0
    requirementsFiles = [ "${config.deps.esp-idf}/tools/requirements/requirements.core.txt" ];
    flattenDependencies = true;
    # expose setuptools as a runtime dep
    buildDependencies.setuptools = false;
  };
}

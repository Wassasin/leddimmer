{ fetchurl, stdenv, lib, autoPatchelfHook, libGL }:
stdenv.mkDerivation {
  pname = "tovrmlx3d";
  version = "4.3.0";
  src = fetchurl {
    url = "https://jenkins.castle-engine.io/public/builds/view3dscene/view3dscene-4.3.0-linux-x86_64.tar.gz";
    hash = "sha256-KAZ90wEJpn71e9fdj4/7FLuFC3Sa/HcNGsjZa406qYI=";
  };
  appendRunpaths = [ "${lib.getLib libGL}/lib" ];
  nativeBuildInputs = [ autoPatchelfHook ];
  installPhase = ''
    mkdir -p $out/bin
    cp tovrmlx3d $out/bin/
  '';
}

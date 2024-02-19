{ kicad-unstable, fetchFromGitLab }:
kicad-unstable.override {
  srcs = {
    kicadVersion = "2023-08-25";
    kicad = fetchFromGitLab {
      group = "kicad";
      owner = "code";
      repo = "kicad";
      rev = "d193334a102bf105121205db7a2716ef30a15519";
      sha256 = "sha256-tzhDmHS5dQ+zi/oUXBDMsXwdvgFq75Z9z3T0uRl/Elo=";
    };
  };
}

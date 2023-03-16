package org.frc1778.lib.pathplanner.simple.parser;

import java.util.List;
import java.util.Map;

public interface ContainerFactory {
  Map createObjectContainer();

  List creatArrayContainer();
}


#include "Maquina/Registry.h"

using namespace maquina;

void register_curveCns( Registry &r );
void register_ikfk2Bone( Registry &r );
void register_intMatrix( Registry &r );
void register_inverseRotOrder( Registry &r );
void register_matrixConstraint( Registry &r );
void register_mulMatrix( Registry &r );
void register_rollSplineKine( Registry &r );
void register_slideCurve2( Registry &r );
void register_springNode( Registry &r );
void register_squashStretch2( Registry &r );

MAQUINA_DECLARE_PLUGINS
MAQUINA_REGISTER_PLUGINS
{
	register_curveCns(r);
  register_ikfk2Bone(r);
  register_intMatrix(r);
  register_inverseRotOrder(r);
	register_matrixConstraint(r),
  register_mulMatrix(r);
  register_rollSplineKine(r);
  register_slideCurve2(r);
  register_springNode(r);
  register_squashStretch2(r);
}

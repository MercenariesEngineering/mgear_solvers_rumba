
#include "Rumba/Rumba.h"

using namespace rumba;

void register_curveCns( Registry &r );
void register_ikfk2Bone( Registry &r );
void register_intMatrix( Registry &r );
void register_inverseRotOrder( Registry &r );
void register_mulMatrix( Registry &r );

RUMBA_DECLARE_PLUGINS
RUMBA_REGISTER_PLUGINS
{
	register_curveCns(r);
  register_ikfk2Bone(r);
  register_intMatrix(r);
  register_inverseRotOrder(r);
  register_mulMatrix(r);
}

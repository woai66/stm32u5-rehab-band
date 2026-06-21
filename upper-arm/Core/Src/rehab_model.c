#include "rehab_model.h"

RehabModelAction_t RehabModel_Predict(const RehabModelFeatures_t *f)
{
  if (f == 0)
  {
    return REHAB_MODEL_IDLE;
  }
  if (f->rule_latch_forearm_rot <= 0.500000f)
  {
    if (f->fore_deg <= 8.645000f)
    {
      if (f->elbow_deg <= 1.570000f)
      {
        return REHAB_MODEL_FOREARM_ROT;
      }
      else
      {
        return REHAB_MODEL_IDLE;
      }
    }
    else
    {
      return REHAB_MODEL_ELBOW_FLEX;
    }
  }
  else
  {
    return REHAB_MODEL_FOREARM_ROT;
  }
}

const char *RehabModel_ActionName(RehabModelAction_t action)
{
  switch (action)
  {
    case REHAB_MODEL_ARM_ABD:
      return "ARM_ABD";
    case REHAB_MODEL_ELBOW_FLEX:
      return "ELBOW_FLEX";
    case REHAB_MODEL_FOREARM_ROT:
      return "FOREARM_ROT";
    case REHAB_MODEL_SHOULDER_LIFT:
      return "SHOULDER_LIFT";
    case REHAB_MODEL_IDLE:
    default:
      return "IDLE";
  }
}

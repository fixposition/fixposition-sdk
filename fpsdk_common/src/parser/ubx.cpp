/**
 * \verbatim
 * ___    ___
 * \  \  /  /
 *  \  \/  /   Copyright (c) Fixposition AG (www.fixposition.com) and contributors
 *  /  /\  \   License: see the LICENSE file
 * /__/  \__\
 *
 * Based on work by flipflip (https://github.com/phkehl)
 * \endverbatim
 *
 * @file
 * @brief Fixposition SDK: Parser UBX routines and types
 */

/* LIBC/STL */
#include <algorithm>
#include <array>
#include <cctype>
#include <cinttypes>
#include <cstddef>
#include <cstdio>
#include <cstring>

/* EXTERNAL */

/* PACKAGE */
#include "fpsdk_common/gnss.hpp"
#include "fpsdk_common/logging.hpp"
#include "fpsdk_common/math.hpp"
#include "fpsdk_common/parser/crc.hpp"
#include "fpsdk_common/parser/types.hpp"
#include "fpsdk_common/parser/ubx.hpp"

namespace fpsdk {
namespace common {
namespace parser {
namespace ubx {
/* ****************************************************************************************************************** */

// clang-format off
// @fp_codegen_begin{FPSDK_COMMON_PARSER_UBX_MSGINFO_CPP}
static const UbxClassesInfo CLS_INFO =
{{
    { UBX_ACK_CLSID,                 0x00,                          UBX_ACK_STRID                  },
    { UBX_CFG_CLSID,                 0x00,                          UBX_CFG_STRID                  },
    { UBX_ESF_CLSID,                 0x00,                          UBX_ESF_STRID                  },
    { UBX_INF_CLSID,                 0x00,                          UBX_INF_STRID                  },
    { UBX_LOG_CLSID,                 0x00,                          UBX_LOG_STRID                  },
    { UBX_MGA_CLSID,                 0x00,                          UBX_MGA_STRID                  },
    { UBX_MON_CLSID,                 0x00,                          UBX_MON_STRID                  },
    { UBX_NAV_CLSID,                 0x00,                          UBX_NAV_STRID                  },
    { UBX_NAV2_CLSID,                0x00,                          UBX_NAV2_STRID                 },
    { UBX_RXM_CLSID,                 0x00,                          UBX_RXM_STRID                  },
    { UBX_SEC_CLSID,                 0x00,                          UBX_SEC_STRID                  },
    { UBX_TIM_CLSID,                 0x00,                          UBX_TIM_STRID                  },
    { UBX_UPD_CLSID,                 0x00,                          UBX_UPD_STRID                  },
    { UBX_NMEA_CLSID,                0x00,                          UBX_NMEA_STRID                 },
    { UBX_RTCM3_CLSID,               0x00,                          UBX_RTCM3_STRID                },
}};
static const UbxMessagesInfo MSG_INFO =
{{
    { UBX_ACK_CLSID,                 UBX_ACK_ACK_MSGID,             UBX_ACK_ACK_STRID              },
    { UBX_ACK_CLSID,                 UBX_ACK_NAK_MSGID,             UBX_ACK_NAK_STRID              },
    { UBX_CFG_CLSID,                 UBX_CFG_CFG_MSGID,             UBX_CFG_CFG_STRID              },
    { UBX_CFG_CLSID,                 UBX_CFG_PWR_MSGID,             UBX_CFG_PWR_STRID              },
    { UBX_CFG_CLSID,                 UBX_CFG_RST_MSGID,             UBX_CFG_RST_STRID              },
    { UBX_CFG_CLSID,                 UBX_CFG_VALDEL_MSGID,          UBX_CFG_VALDEL_STRID           },
    { UBX_CFG_CLSID,                 UBX_CFG_VALGET_MSGID,          UBX_CFG_VALGET_STRID           },
    { UBX_CFG_CLSID,                 UBX_CFG_VALSET_MSGID,          UBX_CFG_VALSET_STRID           },
    { UBX_ESF_CLSID,                 UBX_ESF_ALG_MSGID,             UBX_ESF_ALG_STRID              },
    { UBX_ESF_CLSID,                 UBX_ESF_INS_MSGID,             UBX_ESF_INS_STRID              },
    { UBX_ESF_CLSID,                 UBX_ESF_MEAS_MSGID,            UBX_ESF_MEAS_STRID             },
    { UBX_ESF_CLSID,                 UBX_ESF_RAW_MSGID,             UBX_ESF_RAW_STRID              },
    { UBX_ESF_CLSID,                 UBX_ESF_STATUS_MSGID,          UBX_ESF_STATUS_STRID           },
    { UBX_INF_CLSID,                 UBX_INF_DEBUG_MSGID,           UBX_INF_DEBUG_STRID            },
    { UBX_INF_CLSID,                 UBX_INF_ERROR_MSGID,           UBX_INF_ERROR_STRID            },
    { UBX_INF_CLSID,                 UBX_INF_NOTICE_MSGID,          UBX_INF_NOTICE_STRID           },
    { UBX_INF_CLSID,                 UBX_INF_TEST_MSGID,            UBX_INF_TEST_STRID             },
    { UBX_INF_CLSID,                 UBX_INF_WARNING_MSGID,         UBX_INF_WARNING_STRID          },
    { UBX_LOG_CLSID,                 UBX_LOG_CREATE_MSGID,          UBX_LOG_CREATE_STRID           },
    { UBX_LOG_CLSID,                 UBX_LOG_ERASE_MSGID,           UBX_LOG_ERASE_STRID            },
    { UBX_LOG_CLSID,                 UBX_LOG_FINDTIME_MSGID,        UBX_LOG_FINDTIME_STRID         },
    { UBX_LOG_CLSID,                 UBX_LOG_INFO_MSGID,            UBX_LOG_INFO_STRID             },
    { UBX_LOG_CLSID,                 UBX_LOG_RETR_MSGID,            UBX_LOG_RETR_STRID             },
    { UBX_LOG_CLSID,                 UBX_LOG_RETRPOS_MSGID,         UBX_LOG_RETRPOS_STRID          },
    { UBX_LOG_CLSID,                 UBX_LOG_RETRPOSX_MSGID,        UBX_LOG_RETRPOSX_STRID         },
    { UBX_LOG_CLSID,                 UBX_LOG_RETRSTR_MSGID,         UBX_LOG_RETRSTR_STRID          },
    { UBX_LOG_CLSID,                 UBX_LOG_STR_MSGID,             UBX_LOG_STR_STRID              },
    { UBX_MGA_CLSID,                 UBX_MGA_ACK_MSGID,             UBX_MGA_ACK_STRID              },
    { UBX_MGA_CLSID,                 UBX_MGA_BDS_MSGID,             UBX_MGA_BDS_STRID              },
    { UBX_MGA_CLSID,                 UBX_MGA_DBD_MSGID,             UBX_MGA_DBD_STRID              },
    { UBX_MGA_CLSID,                 UBX_MGA_GAL_MSGID,             UBX_MGA_GAL_STRID              },
    { UBX_MGA_CLSID,                 UBX_MGA_GLO_MSGID,             UBX_MGA_GLO_STRID              },
    { UBX_MGA_CLSID,                 UBX_MGA_GPS_MSGID,             UBX_MGA_GPS_STRID              },
    { UBX_MGA_CLSID,                 UBX_MGA_INI_MSGID,             UBX_MGA_INI_STRID              },
    { UBX_MGA_CLSID,                 UBX_MGA_QZSS_MSGID,            UBX_MGA_QZSS_STRID             },
    { UBX_MON_CLSID,                 UBX_MON_COMMS_MSGID,           UBX_MON_COMMS_STRID            },
    { UBX_MON_CLSID,                 UBX_MON_GNSS_MSGID,            UBX_MON_GNSS_STRID             },
    { UBX_MON_CLSID,                 UBX_MON_HW_MSGID,              UBX_MON_HW_STRID               },
    { UBX_MON_CLSID,                 UBX_MON_HW2_MSGID,             UBX_MON_HW2_STRID              },
    { UBX_MON_CLSID,                 UBX_MON_HW3_MSGID,             UBX_MON_HW3_STRID              },
    { UBX_MON_CLSID,                 UBX_MON_IO_MSGID,              UBX_MON_IO_STRID               },
    { UBX_MON_CLSID,                 UBX_MON_MSGPP_MSGID,           UBX_MON_MSGPP_STRID            },
    { UBX_MON_CLSID,                 UBX_MON_PATCH_MSGID,           UBX_MON_PATCH_STRID            },
    { UBX_MON_CLSID,                 UBX_MON_RF_MSGID,              UBX_MON_RF_STRID               },
    { UBX_MON_CLSID,                 UBX_MON_RXBUF_MSGID,           UBX_MON_RXBUF_STRID            },
    { UBX_MON_CLSID,                 UBX_MON_RXR_MSGID,             UBX_MON_RXR_STRID              },
    { UBX_MON_CLSID,                 UBX_MON_SPAN_MSGID,            UBX_MON_SPAN_STRID             },
    { UBX_MON_CLSID,                 UBX_MON_SYS_MSGID,             UBX_MON_SYS_STRID              },
    { UBX_MON_CLSID,                 UBX_MON_TEMP_MSGID,            UBX_MON_TEMP_STRID             },
    { UBX_MON_CLSID,                 UBX_MON_TXBUF_MSGID,           UBX_MON_TXBUF_STRID            },
    { UBX_MON_CLSID,                 UBX_MON_VER_MSGID,             UBX_MON_VER_STRID              },
    { UBX_NAV_CLSID,                 UBX_NAV_ATT_MSGID,             UBX_NAV_ATT_STRID              },
    { UBX_NAV_CLSID,                 UBX_NAV_CLOCK_MSGID,           UBX_NAV_CLOCK_STRID            },
    { UBX_NAV_CLSID,                 UBX_NAV_COV_MSGID,             UBX_NAV_COV_STRID              },
    { UBX_NAV_CLSID,                 UBX_NAV_DOP_MSGID,             UBX_NAV_DOP_STRID              },
    { UBX_NAV_CLSID,                 UBX_NAV_EELL_MSGID,            UBX_NAV_EELL_STRID             },
    { UBX_NAV_CLSID,                 UBX_NAV_EOE_MSGID,             UBX_NAV_EOE_STRID              },
    { UBX_NAV_CLSID,                 UBX_NAV_GEOFENCE_MSGID,        UBX_NAV_GEOFENCE_STRID         },
    { UBX_NAV_CLSID,                 UBX_NAV_HPPOSECEF_MSGID,       UBX_NAV_HPPOSECEF_STRID        },
    { UBX_NAV_CLSID,                 UBX_NAV_HPPOSLLH_MSGID,        UBX_NAV_HPPOSLLH_STRID         },
    { UBX_NAV_CLSID,                 UBX_NAV_ODO_MSGID,             UBX_NAV_ODO_STRID              },
    { UBX_NAV_CLSID,                 UBX_NAV_ORB_MSGID,             UBX_NAV_ORB_STRID              },
    { UBX_NAV_CLSID,                 UBX_NAV_PL_MSGID,              UBX_NAV_PL_STRID               },
    { UBX_NAV_CLSID,                 UBX_NAV_POSECEF_MSGID,         UBX_NAV_POSECEF_STRID          },
    { UBX_NAV_CLSID,                 UBX_NAV_POSLLH_MSGID,          UBX_NAV_POSLLH_STRID           },
    { UBX_NAV_CLSID,                 UBX_NAV_PVAT_MSGID,            UBX_NAV_PVAT_STRID             },
    { UBX_NAV_CLSID,                 UBX_NAV_PVT_MSGID,             UBX_NAV_PVT_STRID              },
    { UBX_NAV_CLSID,                 UBX_NAV_RELPOSNED_MSGID,       UBX_NAV_RELPOSNED_STRID        },
    { UBX_NAV_CLSID,                 UBX_NAV_RESETODO_MSGID,        UBX_NAV_RESETODO_STRID         },
    { UBX_NAV_CLSID,                 UBX_NAV_SAT_MSGID,             UBX_NAV_SAT_STRID              },
    { UBX_NAV_CLSID,                 UBX_NAV_SBAS_MSGID,            UBX_NAV_SBAS_STRID             },
    { UBX_NAV_CLSID,                 UBX_NAV_SIG_MSGID,             UBX_NAV_SIG_STRID              },
    { UBX_NAV_CLSID,                 UBX_NAV_SLAS_MSGID,            UBX_NAV_SLAS_STRID             },
    { UBX_NAV_CLSID,                 UBX_NAV_SOL_MSGID,             UBX_NAV_SOL_STRID              },
    { UBX_NAV_CLSID,                 UBX_NAV_STATUS_MSGID,          UBX_NAV_STATUS_STRID           },
    { UBX_NAV_CLSID,                 UBX_NAV_SVIN_MSGID,            UBX_NAV_SVIN_STRID             },
    { UBX_NAV_CLSID,                 UBX_NAV_TIMEBDS_MSGID,         UBX_NAV_TIMEBDS_STRID          },
    { UBX_NAV_CLSID,                 UBX_NAV_TIMEGAL_MSGID,         UBX_NAV_TIMEGAL_STRID          },
    { UBX_NAV_CLSID,                 UBX_NAV_TIMEGLO_MSGID,         UBX_NAV_TIMEGLO_STRID          },
    { UBX_NAV_CLSID,                 UBX_NAV_TIMEGPS_MSGID,         UBX_NAV_TIMEGPS_STRID          },
    { UBX_NAV_CLSID,                 UBX_NAV_TIMELS_MSGID,          UBX_NAV_TIMELS_STRID           },
    { UBX_NAV_CLSID,                 UBX_NAV_TIMEQZSS_MSGID,        UBX_NAV_TIMEQZSS_STRID         },
    { UBX_NAV_CLSID,                 UBX_NAV_TIMENAVIC_MSGID,       UBX_NAV_TIMENAVIC_STRID        },
    { UBX_NAV_CLSID,                 UBX_NAV_TIMETRUSTED_MSGID,     UBX_NAV_TIMETRUSTED_STRID      },
    { UBX_NAV_CLSID,                 UBX_NAV_TIMEUTC_MSGID,         UBX_NAV_TIMEUTC_STRID          },
    { UBX_NAV_CLSID,                 UBX_NAV_VELECEF_MSGID,         UBX_NAV_VELECEF_STRID          },
    { UBX_NAV_CLSID,                 UBX_NAV_VELNED_MSGID,          UBX_NAV_VELNED_STRID           },
    { UBX_NAV2_CLSID,                UBX_NAV2_CLOCK_MSGID,          UBX_NAV2_CLOCK_STRID           },
    { UBX_NAV2_CLSID,                UBX_NAV2_COV_MSGID,            UBX_NAV2_COV_STRID             },
    { UBX_NAV2_CLSID,                UBX_NAV2_DOP_MSGID,            UBX_NAV2_DOP_STRID             },
    { UBX_NAV2_CLSID,                UBX_NAV2_EOE_MSGID,            UBX_NAV2_EOE_STRID             },
    { UBX_NAV2_CLSID,                UBX_NAV2_ODO_MSGID,            UBX_NAV2_ODO_STRID             },
    { UBX_NAV2_CLSID,                UBX_NAV2_POSECEF_MSGID,        UBX_NAV2_POSECEF_STRID         },
    { UBX_NAV2_CLSID,                UBX_NAV2_POSLLH_MSGID,         UBX_NAV2_POSLLH_STRID          },
    { UBX_NAV2_CLSID,                UBX_NAV2_PVT_MSGID,            UBX_NAV2_PVT_STRID             },
    { UBX_NAV2_CLSID,                UBX_NAV2_SAT_MSGID,            UBX_NAV2_SAT_STRID             },
    { UBX_NAV2_CLSID,                UBX_NAV2_SBAS_MSGID,           UBX_NAV2_SBAS_STRID            },
    { UBX_NAV2_CLSID,                UBX_NAV2_SIG_MSGID,            UBX_NAV2_SIG_STRID             },
    { UBX_NAV2_CLSID,                UBX_NAV2_SLAS_MSGID,           UBX_NAV2_SLAS_STRID            },
    { UBX_NAV2_CLSID,                UBX_NAV2_STATUS_MSGID,         UBX_NAV2_STATUS_STRID          },
    { UBX_NAV2_CLSID,                UBX_NAV2_SVIN_MSGID,           UBX_NAV2_SVIN_STRID            },
    { UBX_NAV2_CLSID,                UBX_NAV2_TIMEBDS_MSGID,        UBX_NAV2_TIMEBDS_STRID         },
    { UBX_NAV2_CLSID,                UBX_NAV2_TIMEGAL_MSGID,        UBX_NAV2_TIMEGAL_STRID         },
    { UBX_NAV2_CLSID,                UBX_NAV2_TIMEGLO_MSGID,        UBX_NAV2_TIMEGLO_STRID         },
    { UBX_NAV2_CLSID,                UBX_NAV2_TIMEGPS_MSGID,        UBX_NAV2_TIMEGPS_STRID         },
    { UBX_NAV2_CLSID,                UBX_NAV2_TIMELS_MSGID,         UBX_NAV2_TIMELS_STRID          },
    { UBX_NAV2_CLSID,                UBX_NAV2_TIMEQZSS_MSGID,       UBX_NAV2_TIMEQZSS_STRID        },
    { UBX_NAV2_CLSID,                UBX_NAV2_TIMENAVIC_MSGID,      UBX_NAV2_TIMENAVIC_STRID       },
    { UBX_NAV2_CLSID,                UBX_NAV2_TIMEUTC_MSGID,        UBX_NAV2_TIMEUTC_STRID         },
    { UBX_NAV2_CLSID,                UBX_NAV2_VELECEF_MSGID,        UBX_NAV2_VELECEF_STRID         },
    { UBX_NAV2_CLSID,                UBX_NAV2_VELNED_MSGID,         UBX_NAV2_VELNED_STRID          },
    { UBX_RXM_CLSID,                 UBX_RXM_COR_MSGID,             UBX_RXM_COR_STRID              },
    { UBX_RXM_CLSID,                 UBX_RXM_MEASX_MSGID,           UBX_RXM_MEASX_STRID            },
    { UBX_RXM_CLSID,                 UBX_RXM_PMP_MSGID,             UBX_RXM_PMP_STRID              },
    { UBX_RXM_CLSID,                 UBX_RXM_PMREQ_MSGID,           UBX_RXM_PMREQ_STRID            },
    { UBX_RXM_CLSID,                 UBX_RXM_QZSSL6_MSGID,          UBX_RXM_QZSSL6_STRID           },
    { UBX_RXM_CLSID,                 UBX_RXM_RAWX_MSGID,            UBX_RXM_RAWX_STRID             },
    { UBX_RXM_CLSID,                 UBX_RXM_RLM_MSGID,             UBX_RXM_RLM_STRID              },
    { UBX_RXM_CLSID,                 UBX_RXM_RTCM_MSGID,            UBX_RXM_RTCM_STRID             },
    { UBX_RXM_CLSID,                 UBX_RXM_SFRBX_MSGID,           UBX_RXM_SFRBX_STRID            },
    { UBX_RXM_CLSID,                 UBX_RXM_SPARTN_MSGID,          UBX_RXM_SPARTN_STRID           },
    { UBX_RXM_CLSID,                 UBX_RXM_SPARTNKEY_MSGID,       UBX_RXM_SPARTNKEY_STRID        },
    { UBX_SEC_CLSID,                 UBX_SEC_OSNMA_MSGID,           UBX_SEC_OSNMA_STRID            },
    { UBX_SEC_CLSID,                 UBX_SEC_SIG_MSGID,             UBX_SEC_SIG_STRID              },
    { UBX_SEC_CLSID,                 UBX_SEC_SIGLOG_MSGID,          UBX_SEC_SIGLOG_STRID           },
    { UBX_SEC_CLSID,                 UBX_SEC_UNIQID_MSGID,          UBX_SEC_UNIQID_STRID           },
    { UBX_TIM_CLSID,                 UBX_TIM_TM2_MSGID,             UBX_TIM_TM2_STRID              },
    { UBX_TIM_CLSID,                 UBX_TIM_TP_MSGID,              UBX_TIM_TP_STRID               },
    { UBX_TIM_CLSID,                 UBX_TIM_VRFY_MSGID,            UBX_TIM_VRFY_STRID             },
    { UBX_UPD_CLSID,                 UBX_UPD_FLDET_MSGID,           UBX_UPD_FLDET_STRID            },
    { UBX_UPD_CLSID,                 UBX_UPD_POS_MSGID,             UBX_UPD_POS_STRID              },
    { UBX_UPD_CLSID,                 UBX_UPD_SAFEBOOT_MSGID,        UBX_UPD_SAFEBOOT_STRID         },
    { UBX_UPD_CLSID,                 UBX_UPD_SOS_MSGID,             UBX_UPD_SOS_STRID              },
    { UBX_NMEA_CLSID,                UBX_NMEA_STANDARD_DTM_MSGID,   UBX_NMEA_STANDARD_DTM_STRID    },
    { UBX_NMEA_CLSID,                UBX_NMEA_STANDARD_GAQ_MSGID,   UBX_NMEA_STANDARD_GAQ_STRID    },
    { UBX_NMEA_CLSID,                UBX_NMEA_STANDARD_GBQ_MSGID,   UBX_NMEA_STANDARD_GBQ_STRID    },
    { UBX_NMEA_CLSID,                UBX_NMEA_STANDARD_GBS_MSGID,   UBX_NMEA_STANDARD_GBS_STRID    },
    { UBX_NMEA_CLSID,                UBX_NMEA_STANDARD_GGA_MSGID,   UBX_NMEA_STANDARD_GGA_STRID    },
    { UBX_NMEA_CLSID,                UBX_NMEA_STANDARD_GLL_MSGID,   UBX_NMEA_STANDARD_GLL_STRID    },
    { UBX_NMEA_CLSID,                UBX_NMEA_STANDARD_GLQ_MSGID,   UBX_NMEA_STANDARD_GLQ_STRID    },
    { UBX_NMEA_CLSID,                UBX_NMEA_STANDARD_GNQ_MSGID,   UBX_NMEA_STANDARD_GNQ_STRID    },
    { UBX_NMEA_CLSID,                UBX_NMEA_STANDARD_GNS_MSGID,   UBX_NMEA_STANDARD_GNS_STRID    },
    { UBX_NMEA_CLSID,                UBX_NMEA_STANDARD_GPQ_MSGID,   UBX_NMEA_STANDARD_GPQ_STRID    },
    { UBX_NMEA_CLSID,                UBX_NMEA_STANDARD_GQQ_MSGID,   UBX_NMEA_STANDARD_GQQ_STRID    },
    { UBX_NMEA_CLSID,                UBX_NMEA_STANDARD_GRS_MSGID,   UBX_NMEA_STANDARD_GRS_STRID    },
    { UBX_NMEA_CLSID,                UBX_NMEA_STANDARD_GSA_MSGID,   UBX_NMEA_STANDARD_GSA_STRID    },
    { UBX_NMEA_CLSID,                UBX_NMEA_STANDARD_GST_MSGID,   UBX_NMEA_STANDARD_GST_STRID    },
    { UBX_NMEA_CLSID,                UBX_NMEA_STANDARD_GSV_MSGID,   UBX_NMEA_STANDARD_GSV_STRID    },
    { UBX_NMEA_CLSID,                UBX_NMEA_STANDARD_RLM_MSGID,   UBX_NMEA_STANDARD_RLM_STRID    },
    { UBX_NMEA_CLSID,                UBX_NMEA_STANDARD_RMC_MSGID,   UBX_NMEA_STANDARD_RMC_STRID    },
    { UBX_NMEA_CLSID,                UBX_NMEA_STANDARD_TXT_MSGID,   UBX_NMEA_STANDARD_TXT_STRID    },
    { UBX_NMEA_CLSID,                UBX_NMEA_STANDARD_VLW_MSGID,   UBX_NMEA_STANDARD_VLW_STRID    },
    { UBX_NMEA_CLSID,                UBX_NMEA_STANDARD_VTG_MSGID,   UBX_NMEA_STANDARD_VTG_STRID    },
    { UBX_NMEA_CLSID,                UBX_NMEA_STANDARD_ZDA_MSGID,   UBX_NMEA_STANDARD_ZDA_STRID    },
    { UBX_NMEA_CLSID,                UBX_NMEA_PUBX_CONFIG_MSGID,    UBX_NMEA_PUBX_CONFIG_STRID     },
    { UBX_NMEA_CLSID,                UBX_NMEA_PUBX_POSITION_MSGID,  UBX_NMEA_PUBX_POSITION_STRID   },
    { UBX_NMEA_CLSID,                UBX_NMEA_PUBX_RATE_MSGID,      UBX_NMEA_PUBX_RATE_STRID       },
    { UBX_NMEA_CLSID,                UBX_NMEA_PUBX_SVSTATUS_MSGID,  UBX_NMEA_PUBX_SVSTATUS_STRID   },
    { UBX_NMEA_CLSID,                UBX_NMEA_PUBX_TIME_MSGID,      UBX_NMEA_PUBX_TIME_STRID       },
    { UBX_RTCM3_CLSID,               UBX_RTCM3_TYPE1001_MSGID,      UBX_RTCM3_TYPE1001_STRID       },
    { UBX_RTCM3_CLSID,               UBX_RTCM3_TYPE1002_MSGID,      UBX_RTCM3_TYPE1002_STRID       },
    { UBX_RTCM3_CLSID,               UBX_RTCM3_TYPE1003_MSGID,      UBX_RTCM3_TYPE1003_STRID       },
    { UBX_RTCM3_CLSID,               UBX_RTCM3_TYPE1004_MSGID,      UBX_RTCM3_TYPE1004_STRID       },
    { UBX_RTCM3_CLSID,               UBX_RTCM3_TYPE1005_MSGID,      UBX_RTCM3_TYPE1005_STRID       },
    { UBX_RTCM3_CLSID,               UBX_RTCM3_TYPE1006_MSGID,      UBX_RTCM3_TYPE1006_STRID       },
    { UBX_RTCM3_CLSID,               UBX_RTCM3_TYPE1007_MSGID,      UBX_RTCM3_TYPE1007_STRID       },
    { UBX_RTCM3_CLSID,               UBX_RTCM3_TYPE1009_MSGID,      UBX_RTCM3_TYPE1009_STRID       },
    { UBX_RTCM3_CLSID,               UBX_RTCM3_TYPE1010_MSGID,      UBX_RTCM3_TYPE1010_STRID       },
    { UBX_RTCM3_CLSID,               UBX_RTCM3_TYPE1011_MSGID,      UBX_RTCM3_TYPE1011_STRID       },
    { UBX_RTCM3_CLSID,               UBX_RTCM3_TYPE1012_MSGID,      UBX_RTCM3_TYPE1012_STRID       },
    { UBX_RTCM3_CLSID,               UBX_RTCM3_TYPE1033_MSGID,      UBX_RTCM3_TYPE1033_STRID       },
    { UBX_RTCM3_CLSID,               UBX_RTCM3_TYPE1074_MSGID,      UBX_RTCM3_TYPE1074_STRID       },
    { UBX_RTCM3_CLSID,               UBX_RTCM3_TYPE1075_MSGID,      UBX_RTCM3_TYPE1075_STRID       },
    { UBX_RTCM3_CLSID,               UBX_RTCM3_TYPE1077_MSGID,      UBX_RTCM3_TYPE1077_STRID       },
    { UBX_RTCM3_CLSID,               UBX_RTCM3_TYPE1084_MSGID,      UBX_RTCM3_TYPE1084_STRID       },
    { UBX_RTCM3_CLSID,               UBX_RTCM3_TYPE1085_MSGID,      UBX_RTCM3_TYPE1085_STRID       },
    { UBX_RTCM3_CLSID,               UBX_RTCM3_TYPE1087_MSGID,      UBX_RTCM3_TYPE1087_STRID       },
    { UBX_RTCM3_CLSID,               UBX_RTCM3_TYPE1094_MSGID,      UBX_RTCM3_TYPE1094_STRID       },
    { UBX_RTCM3_CLSID,               UBX_RTCM3_TYPE1095_MSGID,      UBX_RTCM3_TYPE1095_STRID       },
    { UBX_RTCM3_CLSID,               UBX_RTCM3_TYPE1097_MSGID,      UBX_RTCM3_TYPE1097_STRID       },
    { UBX_RTCM3_CLSID,               UBX_RTCM3_TYPE1124_MSGID,      UBX_RTCM3_TYPE1124_STRID       },
    { UBX_RTCM3_CLSID,               UBX_RTCM3_TYPE1125_MSGID,      UBX_RTCM3_TYPE1125_STRID       },
    { UBX_RTCM3_CLSID,               UBX_RTCM3_TYPE1127_MSGID,      UBX_RTCM3_TYPE1127_STRID       },
    { UBX_RTCM3_CLSID,               UBX_RTCM3_TYPE1230_MSGID,      UBX_RTCM3_TYPE1230_STRID       },
    { UBX_RTCM3_CLSID,               UBX_RTCM3_TYPE4072_0_MSGID,    UBX_RTCM3_TYPE4072_0_STRID     },
    { UBX_RTCM3_CLSID,               UBX_RTCM3_TYPE4072_1_MSGID,    UBX_RTCM3_TYPE4072_1_STRID     },
}};
// @fp_codegen_end{FPSDK_COMMON_PARSER_UBX_MSGINFO_CPP}
// clang-format on

const UbxClassesInfo& UbxGetClassesInfo()
{
    return CLS_INFO;
}

const UbxMessagesInfo& UbxGetMessagesInfo()
{
    return MSG_INFO;
}

// ---------------------------------------------------------------------------------------------------------------------

bool UbxMessageInfo(const std::string& name, UbxMsgInfo& info)
{
    auto entry =
        std::find_if(MSG_INFO.begin(), MSG_INFO.end(), [name](const auto& cand) { return cand.name_ == name; });
    if (entry != MSG_INFO.end()) {
        info = *entry;
        return true;
    }
    return false;
}

// ---------------------------------------------------------------------------------------------------------------------

static bool UbxMessageName(char* name, const std::size_t size, const uint8_t cls_id, const uint8_t msg_id)
{
    // Check arguments
    if ((name == NULL) || (size < 1)) {
        return false;
    }
    name[0] = '\0';

    std::size_t res = 0;

    // First try the message name lookup table
    for (auto& info : MSG_INFO) {
        if ((info.cls_id_ == cls_id) && (info.msg_id_ == msg_id)) {
            res = std::snprintf(name, size, "%s", info.name_);
            break;
        }
    }

    // If that failed, try the class name lookup table
    if (res == 0) {
        for (auto& info : CLS_INFO) {
            if (info.cls_id_ == cls_id) {
                res = std::snprintf(name, size, "%s-%02" PRIX8, info.name_, msg_id);
                break;
            }
        }
    }

    // If that failed, too, stringify both IDs
    if (res == 0) {
        res = std::snprintf(name, size, "UBX-%02" PRIX8 "-%02" PRIX8, cls_id, msg_id);
    }

    // Did it fit into the string?
    return res < size;
}

bool UbxGetMessageName(char* name, const std::size_t size, const uint8_t* msg, const std::size_t msg_size)
{
    if ((msg == NULL) || (msg_size < UBX_FRAME_SIZE)) {
        return false;
    }
    return UbxMessageName(name, size, UbxClsId(msg), UbxMsgId(msg));
}

// ---------------------------------------------------------------------------------------------------------------------

bool UbxMakeMessage(std::vector<uint8_t>& msg, const uint8_t cls_id, const uint8_t msg_id, const uint8_t* payload,
    const std::size_t payload_size)
{
    if (((payload == NULL) && (payload_size > 0)) || (payload_size > (MAX_UBX_SIZE - UBX_FRAME_SIZE))) {
        return false;
    }

    const uint16_t payload_size_u = payload_size;
    const std::size_t msg_size = payload_size_u + UBX_FRAME_SIZE;
    msg.resize(msg_size);
    msg[0] = UBX_SYNC_1;
    msg[1] = UBX_SYNC_2;
    msg[2] = cls_id;
    msg[3] = msg_id;
    msg[4] = (payload_size_u & 0xff);
    msg[5] = (payload_size_u >> 8);
    if ((payload != NULL) && (payload_size_u > 0)) {
        std::memcpy(&msg[UBX_HEAD_SIZE], payload, payload_size_u);
    }
    const uint16_t ck = crc::ChecksumUbx(&msg[2], msg_size - 2 - 2);
    msg[msg_size - 2] = (ck & 0xff);
    msg[msg_size - 1] = (ck >> 8);
    return true;
}

bool UbxMakeMessage(
    std::vector<uint8_t>& msg, const uint8_t cls_id, const uint8_t msg_id, const std::vector<uint8_t>& payload)
{
    return UbxMakeMessage(msg, cls_id, msg_id, payload.data(), payload.size());
}

// ---------------------------------------------------------------------------------------------------------------------

static const char* UbxSigStr(const uint8_t gnssId, const uint8_t sigId)
{  // clang-format off
    switch (gnssId) {
        case UBX_GNSSID_GPS:
            switch (sigId) {
                case UBX_SIGID_GPS_L1CA: return "L1CA";
                case UBX_SIGID_GPS_L2CL: return "L2CL";
                case UBX_SIGID_GPS_L2CM: return "L2CM";
                case UBX_SIGID_GPS_L5I:  return "L5I";
                case UBX_SIGID_GPS_L5Q:  return "L5Q";
            }
            break;
        case UBX_GNSSID_SBAS:
            switch (sigId) {
                case UBX_SIGID_SBAS_L1CA: return "L1CA";
            }
            break;
        case UBX_GNSSID_GAL:
            switch (sigId) {
                case UBX_SIGID_GAL_E1C:  return "E1C";
                case UBX_SIGID_GAL_E1B:  return "E1B";
                case UBX_SIGID_GAL_E5BI: return "E5BI";
                case UBX_SIGID_GAL_E5BQ: return "E5BQ";
                case UBX_SIGID_GAL_E5AI: return "E5AI";
                case UBX_SIGID_GAL_E5AQ: return "E5AQ";
                case UBX_SIGID_GAL_E6B:  return "E6B";
                case UBX_SIGID_GAL_E6C:  return "E6C";
                case UBX_SIGID_GAL_E6A:  return "E6A";

            }
            break;
        case UBX_GNSSID_BDS:
            switch (sigId) {
                case UBX_SIGID_BDS_B1ID1: return "B1ID1";
                case UBX_SIGID_BDS_B1ID2: return "B1ID2";
                case UBX_SIGID_BDS_B2ID1: return "B2ID1";
                case UBX_SIGID_BDS_B2ID2: return "B2ID2";
                case UBX_SIGID_BDS_B1CP:  return "B1CP";
                case UBX_SIGID_BDS_B1CD:  return "B1CD";
                case UBX_SIGID_BDS_B2AP:  return "B2AP";
                case UBX_SIGID_BDS_B2AD:  return "B2AD";
                case UBX_SIGID_BDS_B3ID1: return "B3ID1";
                case UBX_SIGID_BDS_B3ID2: return "B3ID2";
            }
            break;
        case UBX_GNSSID_QZSS:
            switch (sigId) {
                case UBX_SIGID_QZSS_L1CA: return "L1CA";
                case UBX_SIGID_QZSS_L1S:  return "L1S";
                case UBX_SIGID_QZSS_L2CM: return "L2CM";
                case UBX_SIGID_QZSS_L2CL: return "L2CL";
                case UBX_SIGID_QZSS_L5I:  return "L5I";
                case UBX_SIGID_QZSS_L5Q:  return "L5Q";
            }
            break;
        case UBX_GNSSID_GLO:
            switch (sigId) {
                case UBX_SIGID_GLO_L1OF: return "L1OF";
                case UBX_SIGID_GLO_L2OF: return "L2OF";
            }
            break;
        case UBX_GNSSID_NAVIC:
            switch (sigId) {
                case UBX_SIGID_NAVIC_L5A: return "L5A";
            }
            break;
    }
    return "?";
}  // clang-format on

// ---------------------------------------------------------------------------------------------------------------------

std::size_t UbxRxmSfrbxInfo(char* info, const std::size_t size, const uint8_t* msg, const std::size_t msg_size)
{
    if ((UBX_RXM_SFRBX_VERSION(msg) != UBX_RXM_SFRBX_V2_VERSION) || (msg_size < UBX_RXM_SFRBX_V2_MIN_SIZE)) {
        return 0;
    }

    // Meta information
    UBX_RXM_SFRBX_V2_GROUP0 head;
    std::memcpy(&head, &msg[UBX_HEAD_SIZE], sizeof(head));
    int len = std::snprintf(info, size, "%-4s %-5s ", gnss::UbxGnssIdSvIdToSat(head.gnssId, head.svId).GetStr(),
        UbxSigStr(head.gnssId, head.sigId));

    // Raw sub_frame words
    std::array<uint32_t, 50> dwrd;
    std::size_t n_dwrd = (msg_size - UBX_FRAME_SIZE - sizeof(head)) / sizeof(uint32_t);
    if (n_dwrd == 0) {
        return std::snprintf(&info[len], size - len, "empty");
    }
    std::memcpy(dwrd.data(), &msg[UBX_HEAD_SIZE + sizeof(head)], std::min(dwrd.size(), n_dwrd) * sizeof(uint32_t));
    len += std::snprintf(&info[len], size - len, "%2" PRIuMAX " ", n_dwrd);

    // Sources:
    // - ZED-F9P Integration Manual, 3.13.1 Broadcast navigation data
    // - IS-GPS-200M
    // - Galileo OS SIS ICD
    // - Navipedia

    // GPS L1 C/A LNAV message
    if ((head.gnssId == UBX_GNSSID_GPS) && (head.sigId == UBX_SIGID_GPS_L1CA) && (n_dwrd == 10)) {
        // const uint16_t towCount = (info.dwrd[1] & 0x3fffe000) >> 13;
        const uint8_t sub_frame = (dwrd[1] & 0x00000700) >> 8;
        if ((1 <= sub_frame) && (sub_frame <= 3)) {
            return len + std::snprintf(&info[len], size, "GPS LNAV eph %u/3", sub_frame);
        } else {
            const uint8_t data_id = (dwrd[2] & 0x30000000) >> 28;
            if (data_id == 1) {
                const uint8_t sv_id = (dwrd[2] & 0x0fc00000) >> 22;  // 20.3.3.5.1, 20.3.3.5.1.1 (Table 20-V)
                // sub frame 5 pages 1-24, sub_frame 4 pages 2-5 and 7-10
                if ((1 <= sv_id) && (sv_id <= 32)) {
                    return len + std::snprintf(&info[len], size - len, "GPS LNAV alm %0u", sv_id);
                }
                // sub frame 5, page 25
                else if (sv_id == 51) {
                    return len + std::snprintf(&info[len], size - len,
                                     "GPS LNAV toa, health");  // health 1-24, ALM reference time
                }
                // sub frame 4, page 13
                else if (sv_id == 52) {
                    return len + std::snprintf(&info[len], size - len, "GPS LNAV NCMT");
                }
                // sub frame 4, page 17
                else if (sv_id == 55) {
                    return len + std::snprintf(&info[len], size - len, "GPS LNAV special");
                }
                // sub frame 4, page 18
                else if (sv_id == 56) {
                    return len + std::snprintf(&info[len], size - len, "GPS LNAV iono, UTC");
                }
                // sub frame 4, page 25
                else if (sv_id == 63) {
                    return len + std::snprintf(&info[len], size - len,
                                     "GPS LNAV AS, health");  // anti-spoofing flags, health 25-32
                }
                // reserved
                else {
                    return len + std::snprintf(&info[len], size - len, "GPS LNAV reserved");
                }
            }
        }
        return len + std::snprintf(&info[len], size - len, "bad GPS sf?");  // Hmm.. should we check the parity first?
    }

    // GPS CNAV message (12 seconds, 300 bits)
    else if ((head.gnssId == UBX_GNSSID_GPS) &&
             ((head.sigId == UBX_SIGID_GPS_L2CL) || (head.sigId == UBX_SIGID_GPS_L2CM)) && (n_dwrd == 10)) {
        const uint8_t msg_type_id = (dwrd[0] & 0x0003f000) >> 12;
        const char* const msg_type_descs[] = {
            // clang-format off
            NULL, NULL, NULL, NULL, NULL,
            NULL, NULL, NULL, NULL, NULL,
            /* 10 */ "eph 1", /* 11 */ "eph 2", /* 12 */ "red alm", /* 13 */ "clock diff", /* 14 */ "eph diff",
            /* 15 */ "text", NULL, NULL, NULL, NULL,
            NULL, NULL, NULL, NULL, NULL,
            NULL, NULL, NULL, NULL, NULL,
            /* 30 */ "iono, clk, tgd", /* 31 */ "clock, red. alm", /* 32 */ "clock, EOP", /* 33 */ "clock, UTC", /* 34 */ "clock, diff",
            /* 35 */ "clock, GGTO", /* 36 */ "clock, text", /* 37 */ "clock, mid alm", NULL, NULL,
            /* 40 */ "integrity",
        };  // clang-format on
        return len +
               std::snprintf(&info[len], size - len, "GPS CNAV msg %u %s", msg_type_id,
                   (msg_type_id < (sizeof(msg_type_descs) / sizeof(*msg_type_descs))) && msg_type_descs[msg_type_id]
                       ? msg_type_descs[msg_type_id]
                       : "");
    }

    // GLONASS strings
    else if (head.gnssId == UBX_GNSSID_GLO) {
        // clang-format off
        const uint8_t  string      = (dwrd[0] & 0x78000000) >> 27;
        const uint16_t super_frame = (dwrd[3] & 0xffff0000) >> 16;
        const uint16_t frame       = (dwrd[3] & 0x0000000f);
        // clang-format on
        return len + std::snprintf(&info[len], size - len, "GLO sup %u fr %u str %u", super_frame, frame, string);
    }

    // Galileo I/NAV
    else if ((head.gnssId == UBX_GNSSID_GAL) &&
             ((head.sigId == UBX_SIGID_GAL_E1B) || (head.sigId == UBX_SIGID_GAL_E5BI) ||
                 (head.sigId == UBX_SIGID_GAL_E5BQ))) {
        // FIXME: to check.. the u-blox manual is weird here
        // clang-format off
        const char    even_odd_1  = (dwrd[0] & 0x80000000) == 0x80000000 ? 'o' : 'e'; // even (0) or odd (1)
        const char    page_type_1 = (dwrd[0] & 0x40000000) == 0x40000000 ? 'a' : 'n'; // nominal (0) or alert (1)
        const uint8_t word_type_1 = (dwrd[0] & 0x3f000000) >> 24;
        const char    even_odd_2  = (dwrd[4] & 0x80000000) == 0x80000000 ? 'o' : 'e'; // even (0) or odd (1)
        const char    page_type_2 = (dwrd[4] & 0x40000000) == 0x40000000 ? 'a' : 'n'; // nominal (0) or alert (1)
        const uint8_t word_type_2 = (dwrd[4] & 0x3f000000) >> 24;
        // clang-format on
        return len + std::snprintf(&info[len], size - len, "GAL I/NAV %c %c %u, %c %c %u", page_type_1, even_odd_1,
                         word_type_1, page_type_2, even_odd_2, word_type_2);
    }
    // Galileo F/NAV
    else if ((head.gnssId == UBX_GNSSID_GAL) &&
             ((head.sigId == UBX_SIGID_GAL_E5AI) || (head.sigId == UBX_SIGID_GAL_E5AQ))) {
        const uint8_t pageType = (dwrd[0] & 0xfc000000) >> 26;
        return len + std::snprintf(&info[len], size - len, "GAL F/NAV %u", pageType);
    }

    // unknown
    return len +
           std::snprintf(&info[len], size - len, "unknown (n=%d, 0x%08x, 0x%08x, ...)", (int)n_dwrd, dwrd[0], dwrd[1]);
}

// ---------------------------------------------------------------------------------------------------------------------

std::size_t UbxMonVerToVerStr(char* str, const std::size_t size, const uint8_t* msg, const std::size_t msg_size)
{
    if ((msg == NULL) || (str == NULL) || (size < 2) || (UbxClsId(msg) != UBX_MON_CLSID) ||
        (UbxMsgId(msg) != UBX_MON_VER_MSGID) || (msg_size < UBX_MON_VER_V0_MIN_SIZE)) {
        return false;
    }

    // swVersion: EXT CORE 1.00 (61ce84)
    // hwVersion: 00190000
    // extension: ROM BASE 0xDD3FE36C
    // extension: FWVER=HPG 1.00
    // extension: PROTVER=27.00
    // extension: MOD=ZED-F9P
    // extension: GPS;GLO;GAL;BDS
    // extension: QZSS

    UBX_MON_VER_V0_GROUP0 gr0;
    std::size_t offs = UBX_HEAD_SIZE;
    std::memcpy(&gr0, &msg[offs], sizeof(gr0));
    gr0.swVersion[sizeof(gr0.swVersion) - 1] = '\0';

    char verStr[sizeof(gr0.swVersion)];
    verStr[0] = '\0';
    char modStr[sizeof(gr0.swVersion)];
    modStr[0] = '\0';

    offs += sizeof(gr0);
    UBX_MON_VER_V0_GROUP1 gr1;
    while (offs <= (msg_size - 2 - sizeof(gr1))) {
        std::memcpy(&gr1, &msg[offs], sizeof(gr1));
        if ((verStr[0] == '\0') && (strncmp("FWVER=", gr1.extension, 6) == 0)) {
            gr1.extension[sizeof(gr1.extension) - 1] = '\0';
            std::snprintf(verStr, sizeof(verStr), "%s", &gr1.extension[6]);
        } else if ((modStr[0] == '\0') && (strncmp("MOD=", gr1.extension, 4) == 0)) {
            gr1.extension[sizeof(gr1.extension) - 1] = '\0';
            std::snprintf(modStr, sizeof(modStr), "%s", &gr1.extension[4]);
        }
        offs += sizeof(gr1);
        if ((verStr[0] != '\0') && (modStr[0] != '\0')) {
            break;
        }
    }
    if (verStr[0] == '\0') {
        std::snprintf(verStr, sizeof(verStr), "%s", gr0.swVersion);
    }

    std::size_t len = 0;
    if (modStr[0] != '\0') {
        len = std::snprintf(str, size, "%s (%s)", verStr, modStr);
    } else {
        len = std::snprintf(str, size, "%s", verStr);
    }
    return len;
}

// ---------------------------------------------------------------------------------------------------------------------

static std::size_t StrUbxNav(
    char* info, const std::size_t size, const uint8_t* msg, const std::size_t msg_size, const std::size_t iTowOffs)
{
    if (msg_size < (UBX_FRAME_SIZE + iTowOffs + sizeof(uint32_t))) {
        return 0;
    }
    uint32_t iTOW;
    std::memcpy(&iTOW, &msg[UBX_HEAD_SIZE + iTowOffs], sizeof(iTOW));
    const std::size_t n = std::snprintf(info, size, "%010.3f", (double)iTOW * 1e-3);
    return n;
}

static std::size_t StrUbxNavPvt(char* info, const std::size_t size, const uint8_t* msg, const std::size_t msg_size)
{
    if (msg_size != UBX_NAV_PVT_V1_SIZE) {
        return 0;
    }
    UBX_NAV_PVT_V1_GROUP0 pvt;
    std::memcpy(&pvt, &msg[UBX_HEAD_SIZE], sizeof(pvt));
    int sec = pvt.sec;
    int msec = (pvt.nano / 1000 + 500) / 1000;
    if (msec < 0) {
        sec -= 1;
        msec = 1000 + msec;
    }
    const std::size_t carrSoln = UBX_NAV_PVT_V1_FLAGS_CARRSOLN(pvt.flags);
    constexpr std::array<const char*, 6> fixTypes = { { "nofix", "dr", "2D", "3D", "3D+DR", "time" } };
    const std::size_t n = std::snprintf(info, size,
        "%010.3f"
        " %04u-%02u-%02u (%c)"
        " %02u:%02u:%02d.%03d (%c)"
        " %s (%s, %s)"
        " %2d %4.2f"
        " %+11.7f %+12.7f (%5.1f) %+6.0f (%5.1f)",
        (double)pvt.iTOW * 1e-3, pvt.year, pvt.month, pvt.day,
        UBX_NAV_PVT_V1_VALID_VALIDDATE(pvt.valid) ? (UBX_NAV_PVT_V1_FLAGS2_CONFDATE(pvt.flags2) ? 'Y' : 'y') : 'n',
        pvt.hour, pvt.min, sec, msec,
        UBX_NAV_PVT_V1_VALID_VALIDTIME(pvt.valid) ? (UBX_NAV_PVT_V1_FLAGS2_CONFTIME(pvt.flags2) ? 'Y' : 'y') : 'n',
        pvt.fixType < fixTypes.size() ? fixTypes[pvt.fixType] : "?",
        UBX_NAV_PVT_V1_FLAGS_GNSSFIXOK(pvt.flags) ? "OK" : "masked",
        carrSoln == UBX_NAV_PVT_V1_FLAGS_CARRSOLN_NO
            ? "none"
            : (carrSoln == UBX_NAV_PVT_V1_FLAGS_CARRSOLN_FLOAT
                      ? "float"
                      : (carrSoln == UBX_NAV_PVT_V1_FLAGS_CARRSOLN_FIXED ? "fixed" : "rtk?")),
        pvt.numSV, (double)pvt.pDOP * UBX_NAV_PVT_V1_PDOP_SCALE, (double)pvt.lat * UBX_NAV_PVT_V1_LAT_SCALE,
        (double)pvt.lon * UBX_NAV_PVT_V1_LON_SCALE, (double)pvt.hAcc * UBX_NAV_PVT_V1_HACC_SCALE,
        (double)pvt.height * UBX_NAV_PVT_V1_HEIGHT_SCALE, (double)pvt.vAcc * UBX_NAV_PVT_V1_VACC_SCALE);
    return n;
}

static std::size_t StrUbxNavPosecef(char* info, const std::size_t size, const uint8_t* msg, const std::size_t msg_size)
{
    if (msg_size != UBX_NAV_POSECEF_V0_SIZE) {
        return 0;
    }
    UBX_NAV_POSECEF_V0_GROUP0 pos;
    std::memcpy(&pos, &msg[UBX_HEAD_SIZE], sizeof(pos));
    const std::size_t n = std::snprintf(info, size, "%010.3f %.2f  %.2f  %.2f  %.2f", (double)pos.iTOW * 1e-3,
        (double)pos.ecefX * UBX_NAV_POSECEF_V0_ECEF_XYZ_SCALE, (double)pos.ecefY * UBX_NAV_POSECEF_V0_ECEF_XYZ_SCALE,
        (double)pos.ecefZ * UBX_NAV_POSECEF_V0_ECEF_XYZ_SCALE, (double)pos.pAcc * UBX_NAV_POSECEF_V0_PACC_SCALE);
    return n;
}

static std::size_t StrUbxNavHpposecef(
    char* info, const std::size_t size, const uint8_t* msg, const std::size_t msg_size)
{
    if ((msg_size != UBX_NAV_HPPOSECEF_V0_SIZE) || (UBX_NAV_HPPOSECEF_VERSION(msg) != UBX_NAV_HPPOSECEF_V0_VERSION)) {
        return 0;
    }
    UBX_NAV_HPPOSECEF_V0_GROUP0 pos;
    std::memcpy(&pos, &msg[UBX_HEAD_SIZE], sizeof(pos));
    if (!UBX_NAV_HPPOSECEF_V0_FLAGS_INVALIDECEF(pos.flags)) {
        return std::snprintf(info, size, "%010.3f %.3f %.3f %.3f %.3f", (double)pos.iTOW * 1e-3,
            ((double)pos.ecefX * UBX_NAV_HPPOSECEF_V0_ECEF_XYZ_SCALE) +
                ((double)pos.ecefXHp * UBX_NAV_HPPOSECEF_V0_ECEF_XYZ_HP_SCALE),
            ((double)pos.ecefY * UBX_NAV_HPPOSECEF_V0_ECEF_XYZ_SCALE) +
                ((double)pos.ecefYHp * UBX_NAV_HPPOSECEF_V0_ECEF_XYZ_HP_SCALE),
            ((double)pos.ecefZ * UBX_NAV_HPPOSECEF_V0_ECEF_XYZ_SCALE) +
                ((double)pos.ecefZHp * UBX_NAV_HPPOSECEF_V0_ECEF_XYZ_HP_SCALE),
            (double)pos.pAcc * UBX_NAV_HPPOSECEF_V0_PACC_SCALE);
    } else {
        return std::snprintf(info, size, "%010.3f invalid", (double)pos.iTOW * 1e-3);
    }
}

static std::size_t StrUbxNavRelposned(
    char* info, const std::size_t size, const uint8_t* msg, const std::size_t msg_size)
{
    if ((msg_size != UBX_NAV_RELPOSNED_V1_SIZE) || (UBX_NAV_RELPOSNED_VERSION(msg) != UBX_NAV_RELPOSNED_V1_VERSION)) {
        return 0;
    }
    UBX_NAV_RELPOSNED_V1_GROUP0 rel;
    std::memcpy(&rel, &msg[UBX_HEAD_SIZE], sizeof(rel));
    const std::size_t carrSoln = UBX_NAV_RELPOSNED_V1_FLAGS_CARRSOLN(rel.flags);
    return std::snprintf(info, size,
        "%010.3f N %.3f E %.3f D %.3f L %.3f (%.3f) H %.1f (%.1f) %s %s %s pos:%c head:%c moving:%c posMiss:%c "
        "obsMiss:%c norm:%c",
        (double)rel.iTOW * 1e-3,
        ((double)rel.relPosN * UBX_NAV_RELPOSNED_V1_RELPOSN_E_D_SCALE) +
            ((double)rel.relPosHPN * UBX_NAV_RELPOSNED_V1_RELPOSHPN_E_D_SCALE),
        ((double)rel.relPosE * UBX_NAV_RELPOSNED_V1_RELPOSN_E_D_SCALE) +
            ((double)rel.relPosHPE * UBX_NAV_RELPOSNED_V1_RELPOSHPN_E_D_SCALE),
        ((double)rel.relPosD * UBX_NAV_RELPOSNED_V1_RELPOSN_E_D_SCALE) +
            ((double)rel.relPosHPD * UBX_NAV_RELPOSNED_V1_RELPOSHPN_E_D_SCALE),
        ((double)rel.relPosLength * UBX_NAV_RELPOSNED_V1_RELPOSLENGTH_SCALE) +
            ((double)rel.relPosHPLength * UBX_NAV_RELPOSNED_V1_RELPOSHPLENGTH_SCALE),
        ((double)rel.accLength * UBX_NAV_RELPOSNED_V1_ACCLENGTH_SCALE),
        ((double)rel.relPosHeading * UBX_NAV_RELPOSNED_V1_RELPOSHEADING_SCALE),
        ((double)rel.accHeading * UBX_NAV_RELPOSNED_V1_ACCHEADING_SCALE),
        UBX_NAV_RELPOSNED_V1_FLAGS_GNSSFIXOK(rel.flags) ? "OK" : "masked",
        UBX_NAV_RELPOSNED_V1_FLAGS_DIFFSOLN(rel.flags) ? "diff" : "(diff)",
        carrSoln == 0 ? "none" : (carrSoln == 1 ? "float" : (carrSoln == 2 ? "fixed" : "rtk?")),
        UBX_NAV_RELPOSNED_V1_FLAGS_RELPOSVALID(rel.flags) ? 'Y' : 'N',
        UBX_NAV_RELPOSNED_V1_FLAGS_RELPOSHEADINGVALID(rel.flags) ? 'Y' : 'N',
        UBX_NAV_RELPOSNED_V1_FLAGS_ISMOVING(rel.flags) ? 'Y' : 'N',
        UBX_NAV_RELPOSNED_V1_FLAGS_REFPOSMISS(rel.flags) ? 'Y' : 'N',
        UBX_NAV_RELPOSNED_V1_FLAGS_REFOBSMISS(rel.flags) ? 'Y' : 'N',
        UBX_NAV_RELPOSNED_V1_FLAGS_RELPOSNORMALIZED(rel.flags) ? 'Y' : 'N');
}

static std::size_t StrUbxNavStatus(char* info, const std::size_t size, const uint8_t* msg, const std::size_t msg_size)
{
    if (msg_size != UBX_NAV_STATUS_V0_SIZE) {
        return 0;
    }
    UBX_NAV_STATUS_V0_GROUP0 sta;
    std::memcpy(&sta, &msg[UBX_HEAD_SIZE], sizeof(sta));

    constexpr std::array<const char*, 4> spoofStrs = { { "UNKNOWN", "NONE", "INDICATED", "AFFIRMED" } };
    const auto spoof = UBX_NAV_STATUS_V0_FLAGS2_SPOOFDETSTATE(sta.flags2);
    return std::snprintf(info, size, "%010.3f ttff=%.3f sss=%.3f spoof=%s", (double)sta.iTow * 1e-3,
        (double)sta.ttff * 1e-3, (double)sta.msss * 1e-3, spoof < spoofStrs.size() ? spoofStrs[spoof] : "?");
}

static std::size_t StrUbxNavTimegps(char* info, const std::size_t size, const uint8_t* msg, const std::size_t msg_size)
{
    if (msg_size != UBX_NAV_TIMEGPS_V0_SIZE) {
        return 0;
    }
    UBX_NAV_TIMEGPS_V0_GROUP0 gps;
    std::memcpy(&gps, &msg[UBX_HEAD_SIZE], sizeof(gps));
    return std::snprintf(info, size, "%010.3f %04u %c %019.9f %c %e %d %c",
        (double)gps.iTow * UBX_NAV_TIMEGPS_V0_ITOW_SCALE, gps.week,
        UBX_NAV_TIMEGPS_V0_VALID_WEEKVALID(gps.valid) ? 'Y' : 'N',
        ((double)gps.iTow * UBX_NAV_TIMEGPS_V0_ITOW_SCALE) + ((double)gps.fTOW * UBX_NAV_TIMEGPS_V0_FTOW_SCALE),
        UBX_NAV_TIMEGPS_V0_VALID_TOWVALID(gps.valid) ? 'Y' : 'N', (double)gps.tAcc * UBX_NAV_TIMEGPS_V0_TACC_SCALE,
        gps.leapS, UBX_NAV_TIMEGPS_V0_VALID_LEAPSVALID(gps.valid) ? 'Y' : 'N');
}

static std::size_t StrUbxNavTimegal(char* info, const std::size_t size, const uint8_t* msg, const std::size_t msg_size)
{
    if (msg_size != UBX_NAV_TIMEGAL_V0_SIZE) {
        return 0;
    }
    UBX_NAV_TIMEGAL_V0_GROUP0 gal;
    std::memcpy(&gal, &msg[UBX_HEAD_SIZE], sizeof(gal));
    return std::snprintf(info, size, "%010.3f %04u %c %019.9f %c %e %d %c",
        (double)gal.iTow * UBX_NAV_TIMEGAL_V0_ITOW_SCALE, gal.galWno,
        UBX_NAV_TIMEGAL_V0_VALID_GALWNOVALID(gal.valid) ? 'Y' : 'N',
        (double)gal.galTow + ((double)gal.fGalTow * UBX_NAV_TIMEGAL_V0_FGALTOW_SCALE),
        UBX_NAV_TIMEGAL_V0_VALID_GALTOWVALID(gal.valid) ? 'Y' : 'N', (double)gal.tAcc * UBX_NAV_TIMEGAL_V0_TACC_SCALE,
        gal.leapS, UBX_NAV_TIMEGAL_V0_VALID_LEAPSVALID(gal.valid) ? 'Y' : 'N');
}

static std::size_t StrUbxNavTimebds(char* info, const std::size_t size, const uint8_t* msg, const std::size_t msg_size)
{
    if (msg_size != UBX_NAV_TIMEBDS_V0_SIZE) {
        return 0;
    }
    UBX_NAV_TIMEBDS_V0_GROUP0 bds;
    std::memcpy(&bds, &msg[UBX_HEAD_SIZE], sizeof(bds));
    return std::snprintf(info, size, "%010.3f %04u %c %019.9f %c %e %d %c",
        (double)bds.iTow * UBX_NAV_TIMEBDS_V0_ITOW_SCALE, bds.week,
        UBX_NAV_TIMEBDS_V0_VALID_WEEKVALID(bds.valid) ? 'Y' : 'N',
        (double)bds.SOW + ((double)bds.fSOW * UBX_NAV_TIMEBDS_V0_FSOW_SCALE),
        UBX_NAV_TIMEBDS_V0_VALID_SOWVALID(bds.valid) ? 'Y' : 'N', (double)bds.tAcc * UBX_NAV_TIMEGAL_V0_TACC_SCALE,
        bds.leapS, UBX_NAV_TIMEBDS_V0_VALID_LEAPSVALID(bds.valid) ? 'Y' : 'N');
}

static std::size_t StrUbxNavTimeglo(char* info, const std::size_t size, const uint8_t* msg, const std::size_t msg_size)
{
    if (msg_size != UBX_NAV_TIMEGLO_V0_SIZE) {
        return 0;
    }
    UBX_NAV_TIMEGLO_V0_GROUP0 glo;
    std::memcpy(&glo, &msg[UBX_HEAD_SIZE], sizeof(glo));
    return std::snprintf(info, size, "%010.3f %03u %04u %c %015.9f %c %e",
        (double)glo.iTow * UBX_NAV_TIMEGLO_V0_ITOW_SCALE, glo.N4, glo.Nt,
        UBX_NAV_TIMEGLO_V0_VALID_DATEVALID(glo.valid) ? 'Y' : 'N',
        (double)glo.TOD + ((double)glo.fTOD * UBX_NAV_TIMEGLO_V0_FTOD_SCALE),
        UBX_NAV_TIMEGLO_V0_VALID_TODVALID(glo.valid) ? 'Y' : 'N', (double)glo.tAcc * UBX_NAV_TIMEGAL_V0_TACC_SCALE);
}

static std::size_t StrUbxNavTimeutc(char* info, const std::size_t size, const uint8_t* msg, const std::size_t msg_size)
{
    if (msg_size != UBX_NAV_TIMEUTC_V0_SIZE) {
        return 0;
    }
    UBX_NAV_TIMEUTC_V0_GROUP0 utc;
    std::memcpy(&utc, &msg[UBX_HEAD_SIZE], sizeof(utc));
    const uint8_t utcStandard = UBX_NAV_TIMEUTC_V0_VALID_UTCSTANDARD(utc.valid);
    constexpr std::array<const char*, 9> utcStandardStr = { { "NA", "CRL", "NIST", "USNO", "BIPM", "EU", "SU", "NTSC",
        "NPLI" } };
    return std::snprintf(info, size, "%010.3f %c %04u-%02u-%02u %c %02u:%02u:%02u.%03u %c %s",
        (double)utc.iTow * UBX_NAV_TIMEUTC_V0_ITOW_SCALE, UBX_NAV_TIMEUTC_V0_VALID_VALIDWKN(utc.valid) ? 'Y' : 'N',
        utc.year, utc.month, utc.day, UBX_NAV_TIMEUTC_V0_VALID_VALIDTOW(utc.valid) ? 'Y' : 'N', utc.hour, utc.min,
        utc.sec, (int)(utc.nano * UBX_NAV_TIMEUTC_V0_NANO_SCALE) / 1000000,
        UBX_NAV_TIMEUTC_V0_VALID_VALIDUTC(utc.valid) ? 'Y' : 'N',
        utcStandard < utcStandardStr.size() ? utcStandardStr[utcStandard] : "?");
}

static std::size_t StrUbxNavSig(char* info, const std::size_t size, const uint8_t* msg, const std::size_t msg_size)
{
    if ((msg_size < UBX_NAV_SIG_V0_MIN_SIZE) || (UBX_NAV_SIG_VERSION(msg) != UBX_NAV_SIG_V0_VERSION)) {
        return 0;
    }
    UBX_NAV_SIG_V0_GROUP0 head;
    std::memcpy(&head, &msg[UBX_HEAD_SIZE], sizeof(head));
    return std::snprintf(info, size, "%010.3f %d", (double)head.iTOW * 1e-3, head.numSigs);
}

static std::size_t StrUbxNavSat(char* info, const std::size_t size, const uint8_t* msg, const std::size_t msg_size)
{
    if ((msg_size < UBX_NAV_SAT_V1_MIN_SIZE) || (UBX_NAV_SAT_VERSION(msg) != UBX_NAV_SAT_V1_VERSION)) {
        return 0;
    }
    UBX_NAV_SAT_V1_GROUP0 head;
    std::memcpy(&head, &msg[UBX_HEAD_SIZE], sizeof(head));
    return std::snprintf(info, size, "%010.3f %d", (double)head.iTOW * 1e-3, head.numSvs);
}

static std::size_t StrUbxInf(char* info, const std::size_t size, const uint8_t* msg, const std::size_t msg_size)
{
    if (msg_size < UBX_FRAME_SIZE) {
        return 0;
    }
    const std::size_t infLen = msg_size - UBX_FRAME_SIZE;
    const std::size_t maxLen = size - 1;
    const std::size_t copyLen = std::min(infLen, maxLen);
    std::memcpy(info, &msg[UBX_HEAD_SIZE], copyLen);
    info[copyLen] = '\0';
    for (std::size_t ix = 0; ix < copyLen; ix++) {
        if (std::isprint(info[ix]) == 0) {
            info[ix] = '.';
        }
    }
    return copyLen;
}

static std::size_t StrUbxMonVer(char* info, const std::size_t size, const uint8_t* msg, const std::size_t msg_size)
{
    return UbxMonVerToVerStr(info, size, msg, msg_size);
}

static std::size_t StrUbxMonTemp(char* info, const std::size_t size, const uint8_t* msg, const std::size_t msg_size)
{
    if ((UBX_MON_TEMP_VERSION(msg) != UBX_MON_TEMP_V0_VERSION) || (msg_size != UBX_MON_TEMP_V0_SIZE)) {
        return 0;
    }
    UBX_MON_TEMP_V0_GROUP0 temp;
    std::memcpy(&temp, &msg[UBX_HEAD_SIZE], sizeof(temp));
    return std::snprintf(info, size, "%d C, %d", temp.temperature, temp.unknown);
}

static std::size_t StrUbxMonRf(char* info, const std::size_t size, const uint8_t* msg, const std::size_t msg_size)
{
    if ((UBX_MON_RF_VERSION(msg) != UBX_MON_RF_V0_VERSION) || (msg_size < UBX_MON_RF_V0_MIN_SIZE)) {
        return 0;
    }

    UBX_MON_RF_V0_GROUP0 head;
    std::memcpy(&head, &msg[UBX_HEAD_SIZE], sizeof(head));

    constexpr std::array<const char*, 4> jamStateStrs = { { "UNKN", "OK", "WARN", "CRIT" } };
    constexpr std::array<const char*, 5> antStatusStrs = { { "INIT", "DONTKNOW", "OK", "SHORT", "OPEN" } };
    constexpr std::array<const char*, 3> antPowerStrs = { { "OFF", "ON", "DONTKNOW" } };
    int len = 0;
    int rem = size;
    for (int ix = 0; (ix < head.nBlocks) && (rem > 20); ix++) {
        UBX_MON_RF_V0_GROUP1 rf;
        std::memcpy(&rf, &msg[UBX_HEAD_SIZE + sizeof(head) + (ix * sizeof(rf))], sizeof(rf));
        const uint8_t jamState = UBX_MON_RF_V0_FLAGS_JAMMINGSTATE(rf.flags);
        const float noise = (float)rf.noisePerMS / (float)UBX_MON_RF_V0_NOISEPERMS_MAX * 1e2f;
        const float agc = (float)rf.agcCnt / (float)UBX_MON_RF_V0_AGCCNT_MAX * 1e2f;
        len += std::snprintf(&info[len], rem,
            "%sRF%d (jam %s, ant %s %s, noise %.0f%%, agc %.0f%%, IQ %.1f/%.1f %+.1f/%+.1f)", ix == 0 ? "" : ", ", ix,
            jamState < jamStateStrs.size() ? jamStateStrs[jamState] : "?",
            rf.antStatus < antStatusStrs.size() ? antStatusStrs[rf.antStatus] : "?",
            rf.antPower < antPowerStrs.size() ? antPowerStrs[rf.antPower] : "?", noise, agc,
            (float)rf.magI * (1.0f / 255.0f), (float)rf.magQ * (1.0f / 255.0f), (float)rf.ofsI * (2.0f / 255.0f),
            (float)rf.ofsQ * (2.0f / 255.0f));
        rem = size - len;
    }
    return len;
}

static std::size_t StrUbxMonSys(char* info, const std::size_t size, const uint8_t* msg, const std::size_t msg_size)
{
    if ((UBX_MON_SYS_VERSION(msg) != UBX_MON_SYS_V1_VERSION) || (msg_size != UBX_MON_SYS_V1_SIZE)) {
        return 0;
    }
    UBX_MON_SYS_V1_GROUP0 sys;
    constexpr std::array<const char*, 11> bootTypeStrs = { { "UNKNOWN", "COLDSTART", "WATCHDOG", "HWRESET", "HWBACKUP",
        "SWBACKUP", "SWRESET", "VIOFAIL", "VDDXFAIL", "VDDRFFAIL", "VCOREHIGHFAI" } };
    std::memcpy(&sys, &msg[UBX_HEAD_SIZE], sizeof(sys));
    return std::snprintf(info, size,
        "boot=%s cpu=%" PRIu8 "/%" PRIu8 " mem=%" PRIu8 "/%" PRIu8 " io=%" PRIu8 "/%" PRIu8 " run=%" PRIu32
        " N=%" PRIu16 " W=%" PRIu16 " E=%" PRIu16 " temp=%" PRIi8,
        sys.bootType < bootTypeStrs.size() ? bootTypeStrs[sys.bootType] : "?", sys.cpuLoad, sys.cpuLoadMax,
        sys.memUsage, sys.memUsageMax, sys.ioUsage, sys.ioUsageMax, sys.runTime, sys.noticeCount, sys.warnCount,
        sys.errorCount, sys.tempValue);
}

static std::size_t StrUbxRxmSfrbx(char* info, const std::size_t size, const uint8_t* msg, const std::size_t msg_size)
{
    if ((UBX_RXM_SFRBX_VERSION(msg) != UBX_RXM_SFRBX_V2_VERSION) || (msg_size < UBX_RXM_SFRBX_V2_MIN_SIZE)) {
        return 0;
    }

    if (size < 10) {
        return 0;
    }

    return UbxRxmSfrbxInfo(info, size, msg, msg_size);
}

static std::size_t StrUbxCfgValset(char* info, const std::size_t size, const uint8_t* msg, const std::size_t msg_size)
{
    using namespace fpsdk::common::math;
    if (msg_size < UBX_CFG_VALSET_V0_MIN_SIZE) {
        return 0;
    }
    UBX_CFG_VALSET_V1_GROUP0 /* (~= UBX_CFG_VALGET_V0_GROUP0) */ head;
    std::memcpy(&head, &msg[UBX_HEAD_SIZE], sizeof(head));
    const int dataSize = msg_size - UBX_FRAME_SIZE - sizeof(head);
    char layers[100];
    layers[0] = '\0';
    layers[1] = '\0';
    std::size_t len = 0;
    if (CheckBitsAll(head.layers, UBX_CFG_VALSET_V1_LAYER_RAM)) {
        len += std::snprintf(&layers[len], sizeof(layers) - len, ",RAM");
    }
    if (CheckBitsAll(head.layers, UBX_CFG_VALSET_V1_LAYER_BBR)) {
        len += std::snprintf(&layers[len], sizeof(layers) - len, ",BBR");
    }
    if (CheckBitsAll(head.layers, UBX_CFG_VALSET_V1_LAYER_FLASH)) {
        /* len += */ std::snprintf(&layers[len], sizeof(layers) - len, ",Flash");
    }
    switch (head.version) {
        case UBX_CFG_VALSET_V0_VERSION: {
            return std::snprintf(info, size, "set %d bytes, layers %s", dataSize, &layers[1]);
            break;
        }
        case UBX_CFG_VALSET_V1_VERSION: {
            const char* transaction = "?";
            switch (head.transaction) {
                case UBX_CFG_VALSET_V1_TRANSACTION_NONE:
                    transaction = "NONE";
                    break;
                case UBX_CFG_VALSET_V1_TRANSACTION_BEGIN:
                    transaction = "BEGIN";
                    break;
                case UBX_CFG_VALSET_V1_TRANSACTION_CONTINUE:
                    transaction = "CONTINUE";
                    break;
                case UBX_CFG_VALSET_V1_TRANSACTION_END:
                    transaction = "END";
                    break;
            }
            return std::snprintf(
                info, size, "set %d bytes, layers %s, transaction %s", dataSize, &layers[1], transaction);
            break;
        }
    }
    return 0;
}

static std::size_t StrUbxCfgValdel(char* info, const std::size_t size, const uint8_t* msg, const std::size_t msg_size)
{
    using namespace fpsdk::common::math;
    if ((msg_size < UBX_CFG_VALDEL_V1_MIN_SIZE) || (UBX_CFG_VALDEL_VERSION(msg) != UBX_CFG_VALDEL_V1_VERSION)) {
        return 0;
    }
    UBX_CFG_VALDEL_V1_GROUP0 head;
    std::memcpy(&head, &msg[UBX_HEAD_SIZE], sizeof(head));
    const std::size_t nIds = (msg_size - UBX_FRAME_SIZE - sizeof(head)) / sizeof(uint32_t);
    char layers[100];
    layers[0] = '\0';
    layers[1] = '\0';
    std::size_t len = 0;
    if (CheckBitsAll(head.layers, UBX_CFG_VALDEL_V1_LAYER_BBR)) {
        len += std::snprintf(&layers[len], sizeof(layers) - len, "+BBR");
    }
    if (CheckBitsAll(head.layers, UBX_CFG_VALDEL_V1_LAYER_FLASH)) {
        /* len += */ std::snprintf(&layers[len], sizeof(layers) - len, "+Flash");
    }
    const char* transaction = "?";
    switch (head.transaction) {
        case UBX_CFG_VALDEL_V1_TRANSACTION_NONE:
            transaction = "NONE";
            break;
        case UBX_CFG_VALDEL_V1_TRANSACTION_BEGIN:
            transaction = "BEGIN";
            break;
        case UBX_CFG_VALDEL_V1_TRANSACTION_CONTINUE:
            transaction = "CONTINUE";
            break;
        case UBX_CFG_VALDEL_V1_TRANSACTION_END:
            transaction = "END";
            break;
    }
    return std::snprintf(
        info, size, "del %" PRIuMAX " items, layers %s, transaction %s", nIds, &layers[1], transaction);
}

static const char* _valgetLayerName(const uint8_t layer)
{
    switch (layer) {
        case UBX_CFG_VALGET_V1_LAYER_RAM:
            return "RAM";
        case UBX_CFG_VALGET_V1_LAYER_BBR:
            return "BBR";
        case UBX_CFG_VALGET_V1_LAYER_FLASH:
            return "Flash";
        case UBX_CFG_VALGET_V1_LAYER_DEFAULT:
            return "Default";
    }
    return "?";
}

static std::size_t StrUbxCfgValget(char* info, const std::size_t size, const uint8_t* msg, const std::size_t msg_size)
{
    if (msg_size < UBX_CFG_VALGET_V0_MIN_SIZE) {
        return 0;
    }
    UBX_CFG_VALGET_V0_GROUP0 head;
    std::memcpy(&head, &msg[UBX_HEAD_SIZE], sizeof(head));
    switch (head.version) {
        case UBX_CFG_VALGET_V0_VERSION: {
            const int numKeys = (msg_size - UBX_FRAME_SIZE - sizeof(head)) / sizeof(uint32_t);
            return std::snprintf(info, size, "poll %d items, layer %s, position %u", numKeys,
                _valgetLayerName(head.layer), head.position);
            break;
        }
        case UBX_CFG_VALGET_V1_VERSION: {
            const int dataSize = size - UBX_FRAME_SIZE - sizeof(head);
            return std::snprintf(info, size, "response %d bytes, layer %s, position %u", dataSize,
                _valgetLayerName(head.layer), head.position);
            break;
        }
    }
    return 0;
}

static std::size_t StrUbxAckAck(
    char* info, const std::size_t size, const uint8_t* msg, const std::size_t msg_size, const bool ack)
{
    if (msg_size != UBX_ACK_ACK_V0_SIZE) {
        return 0;
    }
    UBX_ACK_ACK_V0_GROUP0 /* (= UBX_ACK_NAK_V0_GROUP0_t) */ acknak;
    std::memcpy(&acknak, &msg[UBX_HEAD_SIZE], sizeof(acknak));
    char tmp[100];
    UbxMessageName(tmp, sizeof(tmp), acknak.clsId, acknak.msgId);
    return std::snprintf(info, size, "%s %s", ack ? "acknowledgement" : "negative-acknowledgement", tmp);
}

static std::size_t StrUbxTimTm2(char* info, const std::size_t size, const uint8_t* msg, const std::size_t msg_size)
{
    if (msg_size != UBX_TIM_TM2_V0_SIZE) {
        return 0;
    }
    UBX_TIM_TM2_V0_GROUP0 tm;
    std::memcpy(&tm, &msg[UBX_HEAD_SIZE], sizeof(tm));

    const uint8_t timeBase = UBX_TIM_TM2_V0_FLAGS_TIMEBASE(tm.flags);
    constexpr std::array<const char*, 3> timeBaseStr = { { "RX", "GNSS", "UTC" } };
    const double towR =
        ((double)tm.towMsR * UBX_TIM_TM2_V0_TOW_SCALE) + ((double)tm.towSubMsR * UBX_TIM_TM2_V0_SUBMS_SCALE);
    const double towF =
        ((double)tm.towMsF * UBX_TIM_TM2_V0_TOW_SCALE) + ((double)tm.towSubMsF * UBX_TIM_TM2_V0_SUBMS_SCALE);

    return std::snprintf(info, size, "%04u:%013.6f %04u:%013.6f INT%u %c %c %s %s %s %s %s %u %.6g", tm.wnR, towR,
        tm.wnF, towF, tm.ch, UBX_TIM_TM2_V0_FLAGS_NEWRISINGEDGE(tm.flags) ? 'R' : '-',
        UBX_TIM_TM2_V0_FLAGS_NEWFALLINGEDGE(tm.flags) ? 'F' : '-',
        UBX_TIM_TM2_V0_FLAGS_MODE(tm.flags) == UBX_TIM_TM2_V0_FLAGS_MODE_SINGLE ? "single" : "running",
        UBX_TIM_TM2_V0_FLAGS_RUN(tm.flags) == UBX_TIM_TM2_V0_FLAGS_RUN_ARMED ? "armed" : "stopped",
        timeBase < timeBaseStr.size() ? timeBaseStr[timeBase] : "?",
        UBX_TIM_TM2_V0_FLAGS_UTCACAVAIL(tm.flags) ? "UTC" : "n/a",
        UBX_TIM_TM2_V0_FLAGS_TIMEVALID(tm.flags) ? "valid" : "invalid", tm.count,
        (double)tm.accEst * UBX_TIM_TM2_V0_ACCEST_SCALE);
}
static std::size_t StrUbxTimTp(char* info, const std::size_t size, const uint8_t* msg, const std::size_t msg_size)
{
    // clang-format off
    // message   11, dt   63, size   36, UBX      UBX-TIM-TM2          2253:061563.388047 2253:061563.588086 INT0 - F running armed GNSS n/a valid 46887 1.3e-08
    // message   43, dt  558, size   36, UBX      UBX-TIM-TM2          2253:061564.387529 2253:061563.588086 INT0 R - running armed GNSS n/a valid 46888 1.3e-08
    // message   67, dt   64, size   36, UBX      UBX-TIM-TM2          2253:061564.387529 2253:061564.588060 INT0 - F running armed GNSS n/a valid 46888 1.3e-08
    // message   91, dt  556, size   36, UBX      UBX-TIM-TM2          2253:061565.387023 2253:061564.588060 INT0 R - running armed GNSS n/a valid 46889 1.3e-08
    // message  115, dt   67, size   36, UBX      UBX-TIM-TM2          2253:061565.387023 2253:061565.587034 INT0 - F running armed GNSS n/a valid 46889 1.3e-08
    // message  154, dt  558, size   36, UBX      UBX-TIM-TM2          2253:061566.386456 2253:061565.587034 INT0 R - running armed GNSS n/a valid 46890 1.3e-08
    // message  173, dt   66, size   36, UBX      UBX-TIM-TM2          2253:061566.386456 2253:061566.587008 INT0 - F running armed GNSS n/a valid 46890 1.3e-08
    // message  202, dt  569, size   36, UBX      UBX-TIM-TM2          2253:061567.385907 2253:061566.587008 INT0 R - running armed GNSS n/a valid 46891 1.3e-08
    // message  226, dt   59, size   36, UBX      UBX-TIM-TM2          2253:061567.385907 2253:061567.585982 INT0 - F running armed GNSS n/a valid 46891 1.3e-08
    // message  263, dt  566, size   36, UBX      UBX-TIM-TM2          2253:061568.385365 2253:061567.585982 INT0 R - running armed GNSS n/a valid 46892 1.3e-08
    // clang-format on
    if (msg_size != UBX_TIM_TP_V0_SIZE) {
        return 0;
    }
    UBX_TIM_TP_V0_GROUP0 tp;
    std::memcpy(&tp, &msg[UBX_HEAD_SIZE], sizeof(tp));

    const uint8_t timeBase = UBX_TIM_TP_V0_FLAGS_TIMEBASE(tp.flags);
    const double tow =
        ((double)tp.towMs * UBX_TIM_TP_V0_TOWMS_SCALE) + ((double)tp.towSubMs * UBX_TIM_TP_V0_TOWSUBMS_SCALE);
    const uint8_t timeRefGnss = UBX_TIM_TP_V0_REFINFO_TIMEREFGNSS(tp.refInfo);
    constexpr std::array<const char*, 5> timeRefGnssStr = { { "GPS", "GLO", "BDS", "GAL", "NAVIC" } };
    const uint8_t utcStandard = UBX_TIM_TP_V0_REFINFO_UTCSTANDARD(tp.refInfo);
    constexpr std::array<const char*, 9> utcStandardStr = { { "NA", "CRL", "NIST", "USNO", "BIPM", "EU", "SU", "NTSC",
        "NPLI" } };

    std::size_t ret = 0;
    switch (timeBase) {
        case UBX_TIM_TP_V0_FLAGS_TIMEBASE_GNSS:
            ret = std::snprintf(info, size, "%019.12f GNSS %s", tow,
                timeRefGnss < timeRefGnssStr.size() ? timeRefGnssStr[timeRefGnss] : "?");
            break;
        case UBX_TIM_TP_V0_FLAGS_TIMEBASE_UTC:
            ret = std::snprintf(info, size, "%019.12f UTC %s %s", tow,
                UBX_TIM_TP_V0_FLAGS_UTC(tp.flags) ? "avail" : "n/a",
                utcStandard < utcStandardStr.size() ? utcStandardStr[utcStandard] : "?");
            break;
        default:
            ret = std::snprintf(info, size, "%019.12f", tow);
            break;
    }

    if (ret < size) {
        ret += std::snprintf(&info[ret], size - ret, " %d %c %c", tp.qErr,
            UBX_TIM_TP_V0_FLAGS_QERRINVALID(tp.flags) ? 'Y' : 'N',
            UBX_TIM_TP_V0_FLAGS_TPNOTLOCKED(tp.flags) ? 'U' : 'L');
    }

    return ret;
}

static std::size_t StrUbxRxmRawx(char* info, const std::size_t size, const uint8_t* msg, const std::size_t msg_size)
{
    if ((UBX_RXM_RAWX_VERSION(msg) != UBX_RXM_RAWX_V1_VERSION) || (msg_size != UBX_RXM_RAWX_V1_SIZE(msg))) {
        return 0;
    }

    UBX_RXM_RAWX_V1_GROUP0 rawx;
    std::memcpy(&rawx, &msg[UBX_HEAD_SIZE], sizeof(rawx));
    return std::snprintf(info, size, "%010.3f %04u %u", rawx.rcvTow, rawx.week, rawx.numMeas);
}

// ---------------------------------------------------------------------------------------------------------------------

bool UbxGetMessageInfo(char* info, const std::size_t size, const uint8_t* msg, const std::size_t msg_size)
{
    if ((info == NULL) || (size < 1)) {
        return false;
    }

    if ((msg == NULL) || (msg_size < (UBX_HEAD_SIZE + 2))) {
        info[0] = '\0';
        return false;
    }

    if (msg_size == UBX_FRAME_SIZE) {
        const std::size_t len = std::snprintf(info, size, "empty message / poll request");
        return (len > 0) && (len < size);
    }

    const uint8_t cls_id = UbxClsId(msg);
    const uint8_t msg_id = UbxMsgId(msg);
    std::size_t len = 0;
    switch (cls_id) {  // clang-format off
        case UBX_NAV_CLSID: switch (msg_id) {
        case UBX_NAV_PVT_MSGID:          len = StrUbxNavPvt(info, size, msg, msg_size);             break;
        case UBX_NAV_POSECEF_MSGID:      len = StrUbxNavPosecef(info, size, msg, msg_size);         break;
        case UBX_NAV_HPPOSECEF_MSGID:    len = StrUbxNavHpposecef(info, size, msg, msg_size);       break;
        case UBX_NAV_RELPOSNED_MSGID:    len = StrUbxNavRelposned(info, size, msg, msg_size);       break;
        case UBX_NAV_STATUS_MSGID:       len = StrUbxNavStatus(info, size, msg, msg_size);          break;
        case UBX_NAV_SAT_MSGID:          len = StrUbxNavSat(info, size, msg, msg_size);             break;
        case UBX_NAV_SIG_MSGID:          len = StrUbxNavSig(info, size, msg, msg_size);             break;
        case UBX_NAV_TIMEGPS_MSGID:      len = StrUbxNavTimegps(info, size, msg, msg_size);         break;
        case UBX_NAV_TIMEGAL_MSGID:      len = StrUbxNavTimegal(info, size, msg, msg_size);         break;
        case UBX_NAV_TIMEBDS_MSGID:      len = StrUbxNavTimebds(info, size, msg, msg_size);         break;
        case UBX_NAV_TIMEGLO_MSGID:      len = StrUbxNavTimeglo(info, size, msg, msg_size);         break;
        case UBX_NAV_TIMEUTC_MSGID:      len = StrUbxNavTimeutc(info, size, msg, msg_size);         break;
        case UBX_NAV_ORB_MSGID:          /* FALLTHROUGH */
        case UBX_NAV_CLOCK_MSGID:        /* FALLTHROUGH */
        case UBX_NAV_DOP_MSGID:          /* FALLTHROUGH */
        case UBX_NAV_POSLLH_MSGID:       /* FALLTHROUGH */
        case UBX_NAV_VELECEF_MSGID:      /* FALLTHROUGH */
        case UBX_NAV_VELNED_MSGID:       /* FALLTHROUGH */
        case UBX_NAV_EOE_MSGID:          /* FALLTHROUGH */
        case UBX_NAV_GEOFENCE_MSGID:     /* FALLTHROUGH */
        case UBX_NAV_TIMELS_MSGID:       /* FALLTHROUGH */
        case UBX_NAV_COV_MSGID:          len = StrUbxNav(info, size, msg, msg_size, 0);             break;
        case UBX_NAV_SVIN_MSGID:         /* FALLTHROUGH */
        case UBX_NAV_ODO_MSGID:          /* FALLTHROUGH */
        case UBX_NAV_HPPOSLLH_MSGID:     len = StrUbxNav(info, size, msg, msg_size, 4);             break; } break;
        case UBX_INF_CLSID:              len = StrUbxInf(info, size, msg, msg_size);                break;
        case UBX_RXM_CLSID:              switch (msg_id) {
        case UBX_RXM_RAWX_MSGID:         len = StrUbxRxmRawx(info, size, msg, msg_size);            break;
        case UBX_RXM_SFRBX_MSGID:        len = StrUbxRxmSfrbx(info, size, msg, msg_size);           break; } break;
        case UBX_MON_CLSID:              switch (msg_id) {
        case UBX_MON_VER_MSGID:          len = StrUbxMonVer(info, size, msg, msg_size);             break;
        case UBX_MON_TEMP_MSGID:         len = StrUbxMonTemp(info, size, msg, msg_size);            break;
        case UBX_MON_RF_MSGID:           len = StrUbxMonRf(info, size, msg, msg_size);              break;
        case UBX_MON_SYS_MSGID:          len = StrUbxMonSys(info, size, msg, msg_size);             break; } break;
        case UBX_CFG_CLSID:              switch (msg_id) {
        case UBX_CFG_VALSET_MSGID:       len = StrUbxCfgValset(info, size, msg, msg_size);          break;
        case UBX_CFG_VALDEL_MSGID:       len = StrUbxCfgValdel(info, size, msg, msg_size);          break;
        case UBX_CFG_VALGET_MSGID:       len = StrUbxCfgValget(info, size, msg, msg_size);          break; } break;
        case UBX_ACK_CLSID:              switch (msg_id) {
        case UBX_ACK_ACK_MSGID:          len = StrUbxAckAck(info, size, msg, msg_size, true);       break;
        case UBX_ACK_NAK_MSGID:          len = StrUbxAckAck(info, size, msg, msg_size, false);      break; } break;
        case UBX_TIM_CLSID:              switch (msg_id) {
        case UBX_TIM_TM2_MSGID:          len = StrUbxTimTm2(info, size, msg, msg_size);             break;
        case UBX_TIM_TP_MSGID:           len = StrUbxTimTp(info, size, msg, msg_size);              break; } break;
    } // clang-format off

    // @todo insert ellipsis instead of failing in case len > size ?
    return (len > 0) && (len < size);
}

/* ****************************************************************************************************************** */
}  // namespace ubx
}  // namespace parser
}  // namespace common
}  // namespace fpsdk

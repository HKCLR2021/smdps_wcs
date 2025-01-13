#ifndef API_ENDPOINTS_H__
#define API_ENDPOINTS_H__

#include <string>

const std::string mtrl_box_info_url       = "/sort/materialBox/getMaterialBoxInfo";
const std::string mtrl_box_info_by_id_url = "/sort/materialBox/getMaterialBoxInfoById";
const std::string cells_info_by_id_url    = "/sort/cell/CellInfoByMaterialBoxId";
const std::string mtrl_box_amt_url        = "/sort/container/materialBoxAmount";
const std::string new_order_url           = "/sort/order/newOrder";
const std::string order_by_id_url         = "/sort/order/getOrderById";
const std::string dis_result_url          = "/sort/dispenseResult";
const std::string health_url              = "/sort/health";

const std::string api = "/api";
const std::string ver = "/v1";

const std::string scanner = "/scanner";

const std::string health = "/health";
const std::string abnormal_dispensation = "/abnormalDispensation";
const std::string abnormal_device = "/abnormalDevice";
const std::string dispense_request = "/dispenseRequest";
const std::string packaging_request = "/packagingRequest";
const std::string packaging_info = "/packagingMachineInfo";
const std::string order_completion = "/orderCompletion";
const std::string cleaning_mac_scan = "/cleaningMachine";
const std::string mtrl_box_con_scan = "/materialBoxContainer";
const std::string pkg_mac_scan = "/packagingMachines";
const std::string vis_inps_sys_scan = "/visionInspectionSystem";

const std::string cleaning_mac_loc = "cleaning_machine";
const std::string mtrl_box_con_loc = "material_box_container";
const std::string pkg_mac_loc = "packaging_machines";
const std::string vis_inps_sys_loc = "vision_inspection_system";

#endif // API_ENDPOINTS_H__
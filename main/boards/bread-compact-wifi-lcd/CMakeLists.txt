# 新增角度协议映射相关文件
set(SERVO_DEGREE_MAPPER_SRC
    servo_degree_mapper.cc
    servo_degree_mapper_demo.cc
)

idf_component_register(
    SRCS "compact_wifi_board_lcd.cc"
         "servo_360.cc"
         "servo_mcp.cc"
         ${SERVO_DEGREE_MAPPER_SRC}
    INCLUDE_DIRS "."
) 
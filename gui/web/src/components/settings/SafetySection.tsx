import React from "react";
import { Alert, Card, Col, Form, InputNumber, Row, Switch, Typography } from "antd";
import { WarningOutlined } from "@ant-design/icons";
import { useTranslation } from "react-i18next";

const { Text, Paragraph } = Typography;

type Props = {
    values: Record<string, any>;
    onChange: (key: string, value: any) => void;
};

export const SafetySection: React.FC<Props> = ({ values, onChange }) => {
    const { t } = useTranslation();
    return (
        <div>
            <Alert
                type="warning"
                showIcon
                icon={<WarningOutlined />}
                message={t("settingsSafety.alertMessage")}
                description={t("settingsSafety.alertDescription")}
                style={{ marginBottom: 16 }}
            />

            {/* Lift / tilt detection is handled by the STM32 firmware,
                not by ROS2. The previous emergency_stop_on_lift /
                emergency_stop_on_tilt switches were UI-only — no node
                in ROS2 ever read them — so they were removed (audit
                2026-05-12). The firmware always emergency-stops on
                lift/tilt when its physical thresholds are tripped;
                this is not configurable from the GUI. */}

            {/* Temperature */}
            <Card size="small" title={t("settingsSafety.motorTemperatureLimits")} style={{ marginBottom: 16 }}>
                <Paragraph type="secondary" style={{ fontSize: 12, marginBottom: 12 }}>
                    {t("settingsSafety.motorTemperatureLimitsDescription")}
                </Paragraph>
                <Form layout="vertical" size="small">
                    <Row gutter={[16, 0]}>
                        <Col xs={12}>
                            <Form.Item
                                label={<Text style={{ color: "#f5222d", fontSize: 12 }}>{t("settingsSafety.stopAbove")}</Text>}
                                tooltip={t("settingsSafety.stopAboveTooltip")}
                            >
                                <InputNumber
                                    value={values.motor_temp_high_c}
                                    onChange={(v) => onChange("motor_temp_high_c", v)}
                                    min={40} max={120} step={5} precision={0}
                                    style={{ width: "100%" }} addonAfter="C"
                                />
                            </Form.Item>
                        </Col>
                        <Col xs={12}>
                            <Form.Item
                                label={<Text style={{ color: "#52c41a", fontSize: 12 }}>{t("settingsSafety.resumeBelow")}</Text>}
                                tooltip={t("settingsSafety.resumeBelowTooltip")}
                            >
                                <InputNumber
                                    value={values.motor_temp_low_c}
                                    onChange={(v) => onChange("motor_temp_low_c", v)}
                                    min={20} max={80} step={5} precision={0}
                                    style={{ width: "100%" }} addonAfter="C"
                                />
                            </Form.Item>
                        </Col>
                    </Row>
                </Form>
            </Card>

            <Card size="small" title="Rollover and impact detection">
                <Paragraph type="secondary" style={{ fontSize: 12, marginBottom: 12 }}>
                    Shadow mode records what would have tripped but never sends an emergency stop. Save and restart the ROS container for these configuration values to take effect; live tuning remains available through ROS parameters.
                </Paragraph>
                <Form layout="vertical" size="small">
                    <Row gutter={[16, 0]}>
                        <Col xs={12} md={8}><Form.Item label="Supervisor enabled"><Switch checked={values.safety_enabled} onChange={(v) => onChange("safety_enabled", v)} /></Form.Item></Col>
                        <Col xs={12} md={8}><Form.Item label="Shadow mode"><Switch checked={values.safety_shadow_mode} onChange={(v) => onChange("safety_shadow_mode", v)} /></Form.Item></Col>
                        <Col xs={12} md={8}><Form.Item label="Trip on active USB IMU loss"><Switch checked={values.safety_trip_on_imu_stale_when_active} onChange={(v) => onChange("safety_trip_on_imu_stale_when_active", v)} /></Form.Item></Col>
                        <Col xs={12} md={8}><Form.Item label="Rollover trip tilt" tooltip="Higher values reduce false alarms; lower values stop sooner."><InputNumber value={values.safety_tilt_absolute_trip_deg} onChange={(v) => onChange("safety_tilt_absolute_trip_deg", v)} min={1} max={89} step={1} addonAfter="°" style={{ width: "100%" }} /></Form.Item></Col>
                        <Col xs={12} md={8}><Form.Item label="Impact acceleration" tooltip="Used with speed loss or rotation evidence; this alone does not trip."><InputNumber value={values.safety_impact_horizontal_accel_trip_mps2} onChange={(v) => onChange("safety_impact_horizontal_accel_trip_mps2", v)} min={0.1} max={30} step={0.5} addonAfter="m/s²" style={{ width: "100%" }} /></Form.Item></Col>
                    </Row>
                </Form>
            </Card>

            {/* max_obstacle_avoidance_distance moved to the Obstacles
                section (ObstaclesSection.tsx) alongside the other
                obstacle-avoidance knobs. */}
        </div>
    );
};

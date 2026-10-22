CREATE TABLE IF NOT EXISTS detections (
    id           SERIAL PRIMARY KEY,
    robot_id     VARCHAR(50),
    lat          DOUBLE PRECISION NOT NULL,
    lng          DOUBLE PRECISION NOT NULL,
    category     VARCHAR(50) NOT NULL,
    confidence   FLOAT NOT NULL,
    collected    BOOLEAN DEFAULT FALSE,
    detected_at  TIMESTAMP DEFAULT NOW(),
    collected_at TIMESTAMP
);
CREATE INDEX IF NOT EXISTS idx_detections_collected   ON detections(collected);
CREATE INDEX IF NOT EXISTS idx_detections_detected_at ON detections(detected_at);
CREATE INDEX IF NOT EXISTS idx_detections_robot_id    ON detections(robot_id);

CREATE TABLE IF NOT EXISTS robot_status (
    robot_id   VARCHAR(50) PRIMARY KEY,
    lat        DOUBLE PRECISION,
    lng        DOUBLE PRECISION,
    status     VARCHAR(20) DEFAULT 'patrol',
    battery    FLOAT,
    updated_at TIMESTAMP DEFAULT NOW()
);

INSERT INTO robot_status (robot_id, lat, lng, status)
VALUES ('robot_01', 35.945149, 126.682853, 'patrol')
ON CONFLICT (robot_id) DO NOTHING;
ALTER TABLE detections ADD COLUMN IF NOT EXISTS assigned_robot VARCHAR(50);

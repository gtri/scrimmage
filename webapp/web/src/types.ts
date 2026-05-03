export interface Vec3 { x: number; y: number; z: number; }
export interface Quat { w: number; x: number; y: number; z: number; }

export interface EntityDto {
  id: number;
  teamId: number;
  subSwarmId: number;
  type: string;
  active: boolean;
  position: Vec3;
  velocity: Vec3;
  orientation: Quat;
}

export interface FrameDto {
  time: number;
  entities: EntityDto[];
}

export interface Origin { lat: number; lon: number; alt: number; }

export interface MissionStartResponse {
  status: string;
  pid: number;
  mission: string;
  origin: Origin;
  timeWarp?: number | null;
}

export interface TopicSpec {
  network: string;
  topic: string;
  typeName: string;
}

export interface TopicMessage {
  network: string;
  topic: string;
  typeName: string;
  tSim: number;
  payloadJson: string;
}

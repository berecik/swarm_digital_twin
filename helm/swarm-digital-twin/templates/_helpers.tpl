{{/*
Expand the name of the chart.
*/}}
{{- define "swarm.name" -}}
{{- default .Chart.Name .Values.nameOverride | trunc 63 | trimSuffix "-" }}
{{- end }}

{{/*
Create a default fully qualified app name.
*/}}
{{- define "swarm.fullname" -}}
{{- if .Values.fullnameOverride }}
{{- .Values.fullnameOverride | trunc 63 | trimSuffix "-" }}
{{- else }}
{{- $name := default .Chart.Name .Values.nameOverride }}
{{- if contains $name .Release.Name }}
{{- .Release.Name | trunc 63 | trimSuffix "-" }}
{{- else }}
{{- printf "%s-%s" .Release.Name $name | trunc 63 | trimSuffix "-" }}
{{- end }}
{{- end }}
{{- end }}

{{/*
Common labels
*/}}
{{- define "swarm.labels" -}}
helm.sh/chart: {{ include "swarm.name" . }}
app.kubernetes.io/name: {{ include "swarm.name" . }}
app.kubernetes.io/instance: {{ .Release.Name }}
app.kubernetes.io/version: {{ .Chart.AppVersion | quote }}
app.kubernetes.io/managed-by: {{ .Release.Service }}
app.marysia.drone/component: swarm-digital-twin
{{- end }}

{{/*
Selector labels
*/}}
{{- define "swarm.selectorLabels" -}}
app.kubernetes.io/name: {{ include "swarm.name" . }}
app.kubernetes.io/instance: {{ .Release.Name }}
{{- end }}

{{/*
Custom image reference — prepends images.registry when non-empty.
  registry="beret", name="ardupilot-sitl" → "beret/ardupilot-sitl"
  registry="",      name="ardupilot-sitl" → "ardupilot-sitl"
*/}}
{{- define "swarm.customImage" -}}
{{- $registry := .registry -}}
{{- if $registry -}}
{{ $registry }}/{{ .name }}:{{ .tag }}
{{- else -}}
{{ .name }}:{{ .tag }}
{{- end -}}
{{- end }}

{{/*
SITL image reference
*/}}
{{- define "swarm.sitlImage" -}}
{{- include "swarm.customImage" (dict "registry" .Values.images.registry "name" .Values.images.sitl.name "tag" .Values.images.sitl.tag) }}
{{- end }}

{{/*
Companion image reference (swarm-node + perception)
*/}}
{{- define "swarm.companionImage" -}}
{{- include "swarm.customImage" (dict "registry" .Values.images.registry "name" .Values.images.companion.name "tag" .Values.images.companion.tag) }}
{{- end }}

{{/*
Zenoh bridge image reference (third-party, full repository path)
*/}}
{{- define "swarm.zenohBridgeImage" -}}
{{ .Values.images.zenohBridge.repository }}:{{ .Values.images.zenohBridge.tag }}
{{- end }}

{{/*
Zenoh router image reference (third-party, full repository path)
*/}}
{{- define "swarm.zenohRouterImage" -}}
{{ .Values.images.zenohRouter.repository }}:{{ .Values.images.zenohRouter.tag }}
{{- end }}

{{/*
Headless service name for StatefulSet DNS
*/}}
{{- define "swarm.headlessServiceName" -}}
{{ include "swarm.fullname" . }}-headless
{{- end }}

{{/*
Zenoh router service name
*/}}
{{- define "swarm.zenohRouterServiceName" -}}
{{ include "swarm.fullname" . }}-zenoh-router
{{- end }}

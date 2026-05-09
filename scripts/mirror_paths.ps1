# Mirror Kansas Left Fast overcross paths to Kansas Right Fast overcross

$fieldYCenter = 4.035

function Mirror-Y {
    param([double]$y)
    return $fieldYCenter - ($y - $fieldYCenter)
}

function Mirror-Rotation {
    param([double]$deg)
    return -$deg
}

function Mirror-Point {
    param($point)
    return @{
        "x" = $point.x
        "y" = Mirror-Y $point.y
    }
}

function Mirror-Path {
    param([string]$inputPath, [string]$outputPath)
    
    $path = Get-Content $inputPath | ConvertFrom-Json
    
    # Mirror waypoints
    for ($i = 0; $i -lt $path.waypoints.Count; $i++) {
        if ($path.waypoints[$i].anchor) {
            $path.waypoints[$i].anchor = Mirror-Point $path.waypoints[$i].anchor
        }
        if ($path.waypoints[$i].prevControl) {
            $path.waypoints[$i].prevControl = Mirror-Point $path.waypoints[$i].prevControl
        }
        if ($path.waypoints[$i].nextControl) {
            $path.waypoints[$i].nextControl = Mirror-Point $path.waypoints[$i].nextControl
        }
    }
    
    # Mirror rotation targets
    for ($i = 0; $i -lt $path.rotationTargets.Count; $i++) {
        $path.rotationTargets[$i].rotationDegrees = Mirror-Rotation $path.rotationTargets[$i].rotationDegrees
    }
    
    # Mirror goal end state rotation
    if ($path.goalEndState) {
        $path.goalEndState.rotation = Mirror-Rotation $path.goalEndState.rotation
    }
    
    # Mirror ideal starting state rotation
    if ($path.idealStartingState) {
        $path.idealStartingState.rotation = Mirror-Rotation $path.idealStartingState.rotation
    }
    
    $path | ConvertTo-Json -Depth 100 | Set-Content $outputPath
    Write-Host "Created $outputPath"
}

# Source and destination paths
$leftPath1 = "src/main/deploy/pathplanner/paths/KS.L.1.oc.path"
$rightPath1 = "src/main/deploy/pathplanner/paths/KS.R.1.oc.path"
Mirror-Path $leftPath1 $rightPath1

$leftPath2 = "src/main/deploy/pathplanner/paths/KS.L.2.f.oc.path"
$rightPath2 = "src/main/deploy/pathplanner/paths/KS.R.2.f.oc.path"
Mirror-Path $leftPath2 $rightPath2

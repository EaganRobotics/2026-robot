param(
    [Parameter(ValueFromRemainingArguments = $true)]
    [string[]]$ScriptArgs
)

$pythonLauncher = Get-Command py -ErrorAction SilentlyContinue
if ($null -ne $pythonLauncher) {
    & $pythonLauncher.Source -3 "$PSScriptRoot\generate_shooter_setpoints.py" @ScriptArgs
    exit $LASTEXITCODE
}

$python = Get-Command python -ErrorAction SilentlyContinue
if ($null -ne $python) {
    & $python.Source "$PSScriptRoot\generate_shooter_setpoints.py" @ScriptArgs
    exit $LASTEXITCODE
}

Write-Error "Python 3 was not found. Install Python or run the script on a machine with 'py' or 'python' available."
exit 1

param(
    [Parameter(Mandatory = $false)]
    [string]$MarkdownPath = "",

    [Parameter(Mandatory = $false)]
    [string]$ProjectName = "M2614 La face cachée de la lune",

    [Parameter(Mandatory = $false)]
    [string]$OutputPdfPath = "",

    [switch]$KeepTemp
)

$ErrorActionPreference = "Stop"

$repoRoot = $PSScriptRoot
if ([string]::IsNullOrWhiteSpace($repoRoot)) {
    $repoRoot = (Get-Location).Path
}

if ([string]::IsNullOrWhiteSpace($MarkdownPath)) {
    $MarkdownPath = Join-Path $repoRoot "docs\document-transmission.md"
}

try {
    $MarkdownPath = (Resolve-Path -LiteralPath $MarkdownPath).Path
}
catch {
    throw "Markdown file not found: $MarkdownPath"
}

if (-not (Test-Path -LiteralPath $MarkdownPath)) {
    throw "Markdown file not found: $MarkdownPath"
}

$doxygen = Get-Command doxygen -ErrorAction Stop

$mdFileName = [System.IO.Path]::GetFileName($MarkdownPath)
$mdDir = [System.IO.Path]::GetDirectoryName($MarkdownPath)
$imagesDir = Join-Path $mdDir "images"

if ([string]::IsNullOrWhiteSpace($OutputPdfPath)) {
    $OutputPdfPath = [System.IO.Path]::ChangeExtension($MarkdownPath, ".pdf")
}

$tmpRoot = Join-Path $env:TEMP ("doxygen-single-" + [guid]::NewGuid().ToString())
New-Item -ItemType Directory -Path $tmpRoot | Out-Null
$outDir = Join-Path $tmpRoot "out"
$doxyfilePath = Join-Path $tmpRoot "Doxyfile"

$doxy = @"
PROJECT_NAME            = "$ProjectName"
OUTPUT_DIRECTORY        = "$outDir"
INPUT                   = "$MarkdownPath"
FILE_PATTERNS           = *.md
RECURSIVE               = NO
GENERATE_HTML           = NO
GENERATE_LATEX          = YES
MARKDOWN_SUPPORT        = YES
USE_PDFLATEX            = YES
QUIET                   = NO
IMAGE_PATH              = "$imagesDir"
"@

$utf8NoBom = New-Object System.Text.UTF8Encoding($false)
[System.IO.File]::WriteAllText($doxyfilePath, $doxy, $utf8NoBom)

& $doxygen.Source $doxyfilePath

$latexDir = Join-Path $outDir "latex"
if (-not (Test-Path -LiteralPath $latexDir)) {
    throw "LaTeX output folder not found: $latexDir"
}

Push-Location $latexDir
try {
    if (Test-Path -LiteralPath (Join-Path $latexDir "make.bat")) {
        & (Join-Path $latexDir "make.bat")
    } elseif (Get-Command make -ErrorAction SilentlyContinue) {
        make
    } elseif (Get-Command latexmk -ErrorAction SilentlyContinue) {
        latexmk -pdf refman.tex
    } else {
        throw "No LaTeX build command found (make.bat, make, or latexmk)."
    }
}
finally {
    Pop-Location
}

$pdfSource = Join-Path $latexDir "refman.pdf"
if (-not (Test-Path -LiteralPath $pdfSource)) {
    throw "PDF not generated: $pdfSource"
}

Copy-Item -LiteralPath $pdfSource -Destination $OutputPdfPath -Force
Write-Host "PDF generated: $OutputPdfPath"

if ($KeepTemp) {
    Write-Host "Temporary folder kept: $tmpRoot"
} else {
    Remove-Item -LiteralPath $tmpRoot -Recurse -Force
}

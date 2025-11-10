# 🚀 Part 2 Assignment — Setup & Execution Guide (Windows)

## 📋 Prerequisites

- **Python Version**: 3.10+ (recommended: Python 3.11)  
- **Dependencies**: No external libraries required  
- **Operating System**: Windows  

> ✅ This submission was done using the free demo version.

---

## 🔧 Initial Setup

1. Open **Command Prompt** or **PowerShell**.
2. Navigate to the project directory:

```bash
cd part2_assignment
```

# 🚀 Running the programs

Run the factory optimizer using JSON input/output redirection:
```bash
python factory/main.py < input_factory.json > output_factory.json
```
# ⚙️ Belts Module
Run the belts/conveyor solver:
```bash
python belts/main.py < input_belts.json > output_belts.json
```
# 🔄 Combined Sample Run
Run both modules using the provided sample harness:
```bash
python run_samples.py "python factory/main.py" "python belts/main.py"
```
# 🧪 Running Tests
Run All Tests
```bash
python -m pytest tests/ -v
```
Run Only Factory Tests
```bash
python -m pytest tests/test_factory.py -v
```
Run Only Belts Tests
```bash
python -m pytest tests/test_belts.py -v
```
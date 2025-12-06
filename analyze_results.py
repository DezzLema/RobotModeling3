#!/usr/bin/env python3
"""
analyze_results.py - Анализатор результатов экспериментов
УПРОЩЕННАЯ ВЕРСИЯ для работы с корректным CSV
"""

import csv
import json
import statistics
import matplotlib.pyplot as plt
import os
import numpy as np

def read_csv_data(filename):
    """Чтение данных из CSV файла"""
    data = []
    if not os.path.exists(filename):
        print(f"❌ Файл {filename} не найден!")
        return data
    
    try:
        with open(filename, 'r') as f:
            reader = csv.DictReader(f)
            
            for i, row in enumerate(reader):
                try:
                    # Преобразуем числовые значения
                    numeric_row = {}
                    for key, value in row.items():
                        if key == 'experiment_id':
                            numeric_row[key] = value.strip()
                        else:
                            try:
                                numeric_row[key] = float(value) if value else 0.0
                            except (ValueError, TypeError):
                                numeric_row[key] = 0.0
                    
                    data.append(numeric_row)
                    print(f"✅ Строка {i+1}: {numeric_row['experiment_id']}")
                    
                except Exception as e:
                    print(f"⚠️ Ошибка в строке {i+1}: {e}")
                    continue
                    
    except Exception as e:
        print(f"❌ Ошибка чтения файла: {e}")
    
    return data

def print_data_summary(data):
    """Вывод сводки по данным"""
    if not data:
        print("Нет данных для анализа")
        return
    
    print(f"\n📊 СВОДКА ПО ДАННЫМ:")
    print("=" * 50)
    print(f"Всего экспериментов: {len(data)}")
    
    print(f"\n📋 СПИСОК ЭКСПЕРИМЕНТОВ:")
    for i, exp in enumerate(data, 1):
        print(f"{i:2d}. {exp['experiment_id']:20s} "
              f"скорость={exp.get('forward_speed', 0):.2f} "
              f"площадь={exp.get('area_coverage', 0):.2f}м²")

def create_basic_plots(data):
    """Создание базовых графиков"""
    if len(data) < 2:
        print("\n⚠️ Для графиков нужно минимум 2 эксперимента")
        return
    
    # Сортируем по ID эксперимента
    data_sorted = sorted(data, key=lambda x: x['experiment_id'])
    
    # График 1: Все эксперименты
    fig, axes = plt.subplots(2, 2, figsize=(14, 10))
    fig.suptitle('РЕЗУЛЬТАТЫ ЭКСПЕРИМЕНТОВ', fontsize=16, fontweight='bold')
    
    # Подготовка данных
    exp_ids = [exp['experiment_id'] for exp in data_sorted]
    areas = [exp.get('area_coverage', 0) for exp in data_sorted]
    speeds = [exp.get('forward_speed', 0) for exp in data_sorted]
    success_rates = [exp.get('avoidance_success_rate', 0) * 100 for exp in data_sorted]
    avoidances = [exp.get('total_avoidances', 0) for exp in data_sorted]
    
    # График 1: Покрытая площадь
    ax1 = axes[0, 0]
    bars1 = ax1.bar(range(len(exp_ids)), areas, color='lightblue', edgecolor='black', alpha=0.8)
    ax1.set_xlabel('Эксперимент')
    ax1.set_ylabel('Покрытая площадь (м²)')
    ax1.set_title('Эффективность исследования')
    ax1.set_xticks(range(len(exp_ids)))
    ax1.set_xticklabels(exp_ids, rotation=45, ha='right', fontsize=9)
    ax1.grid(True, alpha=0.3, axis='y')
    
    # Добавляем значения на столбцы
    for bar, area in zip(bars1, areas):
        height = bar.get_height()
        ax1.text(bar.get_x() + bar.get_width()/2., height + 0.01,
                f'{area:.2f}', ha='center', va='bottom', fontsize=8, fontweight='bold')
    
    # График 2: Успешность объездов
    ax2 = axes[0, 1]
    bars2 = ax2.bar(range(len(exp_ids)), success_rates, color='lightgreen', edgecolor='black', alpha=0.8)
    ax2.set_xlabel('Эксперимент')
    ax2.set_ylabel('Успешность объездов (%)')
    ax2.set_title('Надежность алгоритма')
    ax2.set_xticks(range(len(exp_ids)))
    ax2.set_xticklabels(exp_ids, rotation=45, ha='right', fontsize=9)
    ax2.grid(True, alpha=0.3, axis='y')
    ax2.set_ylim([0, 105])
    
    # Добавляем значения на столбцы
    for bar, success in zip(bars2, success_rates):
        height = bar.get_height()
        ax2.text(bar.get_x() + bar.get_width()/2., height + 1,
                f'{success:.1f}%', ha='center', va='bottom', fontsize=8, fontweight='bold')
    
    # График 3: Количество объездов
    ax3 = axes[1, 0]
    bars3 = ax3.bar(range(len(exp_ids)), avoidances, color='lightcoral', edgecolor='black', alpha=0.8)
    ax3.set_xlabel('Эксперимент')
    ax3.set_ylabel('Количество объездов')
    ax3.set_title('Активность алгоритма')
    ax3.set_xticks(range(len(exp_ids)))
    ax3.set_xticklabels(exp_ids, rotation=45, ha='right', fontsize=9)
    ax3.grid(True, alpha=0.3, axis='y')
    
    # Добавляем значения на столбцы
    for bar, count in zip(bars3, avoidances):
        height = bar.get_height()
        ax3.text(bar.get_x() + bar.get_width()/2., height + 0.1,
                f'{int(count)}', ha='center', va='bottom', fontsize=8, fontweight='bold')
    
    # График 4: Скорость vs Площадь (рассеяние)
    ax4 = axes[1, 1]
    scatter = ax4.scatter(speeds, areas, s=100, c=success_rates, 
                         cmap='RdYlGn', edgecolors='black', alpha=0.7)
    ax4.set_xlabel('Скорость (м/с)')
    ax4.set_ylabel('Покрытая площадь (м²)')
    ax4.set_title('Зависимость: Скорость → Площадь')
    ax4.grid(True, alpha=0.3)
    
    # Добавляем подписи точек
    for i, (x, y, exp_id) in enumerate(zip(speeds, areas, exp_ids)):
        ax4.annotate(exp_id, (x, y), textcoords="offset points", 
                    xytext=(0,10), ha='center', fontsize=8)
    
    plt.colorbar(scatter, ax=ax4, label='Успешность (%)')
    
    plt.tight_layout()
    plt.savefig('all_experiments_analysis.png', dpi=150, bbox_inches='tight')
    print("📊 График 'all_experiments_analysis.png' сохранен")
    plt.show()

def generate_report(data):
    """Генерация отчета"""
    if not data:
        return "Нет данных для отчета"
    
    areas = [exp.get('area_coverage', 0) for exp in data]
    success_rates = [exp.get('avoidance_success_rate', 0) * 100 for exp in data]
    collisions = [exp.get('collisions', 0) for exp in data]
    
    best_exp = max(data, key=lambda x: x.get('area_coverage', 0))
    
    report = f"""
{'='*70}
ОТЧЕТ ПО ОПТИМИЗАЦИИ АЛГОРИТМА БРАЙТЕНБЕРГА
{'='*70}

ОБЩАЯ СТАТИСТИКА:
Всего экспериментов: {len(data)}
Средняя покрытая площадь: {statistics.mean(areas):.2f} м²
Максимальная площадь: {max(areas):.2f} м² (эксперимент: {best_exp['experiment_id']})
Минимальная площадь: {min(areas):.2f} м²
Средняя успешность объездов: {statistics.mean(success_rates):.1f}%
Максимальная успешность: {max(success_rates):.1f}%
Общее количество столкновений: {sum(collisions)}

РЕЗУЛЬТАТЫ ОПТИМИЗАЦИИ:
• Лучший эксперимент: {best_exp['experiment_id']}
• Оптимальная скорость: {best_exp.get('forward_speed', 0):.2f} м/с
• Оптимальная угловая скорость: {best_exp.get('turn_speed', 0):.1f} рад/с
• Оптимальная дистанция остановки: {best_exp.get('stop_distance', 0):.2f} м
• Длительность маневра: {best_exp.get('escape_duration', 0):.1f} с

ВЫВОДЫ:
Проведена оптимизация параметров алгоритма Брайтенберга.
Найденные параметры обеспечивают баланс между эффективностью
исследования и безопасностью движения робота.
"""
    
    return report

def main():
    print("🔍 АНАЛИЗАТОР РЕЗУЛЬТАТОВ ЭКСПЕРИМЕНТОВ")
    print("=" * 60)
    
    # Чтение данных
    print("📁 Чтение данных из CSV файла...")
    data = read_csv_data("all_experiments_summary.csv")
    
    if not data:
        print("\n❌ Нет данных для анализа!")
        print("Создайте CSV файл: python3 lab3.py --mode clean_csv")
        return
    
    # Вывод сводки
    print_data_summary(data)
    
    # Создание графиков
    print("\n📈 Создание графиков...")
    create_basic_plots(data)
    
    # Генерация отчета
    print("\n📝 ГЕНЕРАЦИЯ ОТЧЕТА...")
    report = generate_report(data)
    
    # Сохранение отчета
    with open('optimization_report.txt', 'w', encoding='utf-8') as f:
        f.write(report)
    
    print(report)
    
    # Сохранение оптимальных параметров
    if data:
        best_exp = max(data, key=lambda x: x.get('area_coverage', 0))
        optimal_config = {
            'optimal_parameters': {
                'forward_speed': best_exp.get('forward_speed', 0.15),
                'turn_speed': best_exp.get('turn_speed', 1.5),
                'stop_distance': best_exp.get('stop_distance', 0.3),
                'escape_duration': best_exp.get('escape_duration', 1.5)
            },
            'best_experiment': {
                'id': best_exp['experiment_id'],
                'area_coverage': best_exp.get('area_coverage', 0),
                'avoidance_success_rate': best_exp.get('avoidance_success_rate', 0)
            }
        }
        
        with open('optimal_parameters.json', 'w') as f:
            json.dump(optimal_config, f, indent=2)
        
        print(f"\n💾 Оптимальная конфигурация сохранена в optimal_parameters.json")
    
    print(f"\n✅ АНАЛИЗ ЗАВЕРШЕН УСПЕШНО!")
    print(f"📊 Графики сохранены в all_experiments_analysis.png")
    print(f"📝 Отчет сохранен в optimization_report.txt")

if __name__ == '__main__':
    main()

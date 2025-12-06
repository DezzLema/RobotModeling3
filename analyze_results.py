#!/usr/bin/env python3
"""
analyze_results_simple.py - Упрощённый анализатор результатов
"""

import csv
import json
import statistics
import matplotlib.pyplot as plt
import os
import numpy as np
from datetime import datetime

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
                    numeric_row = {}
                    for key, value in row.items():
                        if key == 'experiment_id':
                            numeric_row[key] = value.strip()
                        else:
                            try:
                                numeric_row[key] = float(value) if value else 0.0
                            except:
                                numeric_row[key] = 0.0
                    
                    data.append(numeric_row)
                    print(f"✅ Строка {i+1}: {numeric_row['experiment_id']}")
                    
                except Exception as e:
                    print(f"⚠️ Ошибка в строке {i+1}: {e}")
                    continue
                    
    except Exception as e:
        print(f"❌ Ошибка чтения файла: {e}")
    
    return data

def create_basic_plots(data):
    """Создание базовых графиков"""
    if len(data) < 2:
        print("\n⚠️ Для графиков нужно минимум 2 эксперимента")
        return
    
    # Сортируем по скорости
    data_sorted = sorted(data, key=lambda x: x['forward_speed'])
    
    # Подготовка данных
    exp_ids = [exp['experiment_id'] for exp in data_sorted]
    areas = [exp.get('area_coverage', 0) for exp in data_sorted]
    speeds = [exp.get('forward_speed', 0) for exp in data_sorted]
    success_rates = [exp.get('avoidance_success_rate', 0) * 100 for exp in data_sorted]
    avoidances = [exp.get('total_avoidances', 0) for exp in data_sorted]
    min_distances = [exp.get('min_obstacle_distance', 0) for exp in data_sorted]
    
    # Создание фигуры
    fig, axes = plt.subplots(2, 2, figsize=(14, 10))
    fig.suptitle('АНАЛИЗ РЕЗУЛЬТАТОВ ЭКСПЕРИМЕНТОВ', fontsize=16, fontweight='bold')
    
    # График 1: Покрытая площадь
    ax1 = axes[0, 0]
    bars1 = ax1.bar(range(len(exp_ids)), areas, color='lightblue', 
                    edgecolor='black', alpha=0.8)
    ax1.set_xlabel('Эксперимент')
    ax1.set_ylabel('Покрытая площадь (м²)')
    ax1.set_title('Эффективность исследования')
    ax1.set_xticks(range(len(exp_ids)))
    ax1.set_xticklabels(exp_ids, rotation=45, ha='right', fontsize=9)
    ax1.grid(True, alpha=0.3, axis='y')
    
    for bar, area in zip(bars1, areas):
        height = bar.get_height()
        ax1.text(bar.get_x() + bar.get_width()/2., height + 0.01,
                f'{area:.2f}', ha='center', va='bottom', fontsize=8, fontweight='bold')
    
    # График 2: Успешность объездов
    ax2 = axes[0, 1]
    bars2 = ax2.bar(range(len(exp_ids)), success_rates, color='lightgreen', 
                    edgecolor='black', alpha=0.8)
    ax2.set_xlabel('Эксперимент')
    ax2.set_ylabel('Успешность объездов (%)')
    ax2.set_title('Надежность алгоритма')
    ax2.set_xticks(range(len(exp_ids)))
    ax2.set_xticklabels(exp_ids, rotation=45, ha='right', fontsize=9)
    ax2.grid(True, alpha=0.3, axis='y')
    ax2.set_ylim([0, 105])
    
    for bar, success in zip(bars2, success_rates):
        height = bar.get_height()
        ax2.text(bar.get_x() + bar.get_width()/2., height + 1,
                f'{success:.1f}%', ha='center', va='bottom', fontsize=8, fontweight='bold')
    
    # График 3: Безопасность
    ax3 = axes[1, 0]
    bars3 = ax3.bar(range(len(exp_ids)), min_distances, color='orange', 
                    edgecolor='black', alpha=0.8)
    ax3.set_xlabel('Эксперимент')
    ax3.set_ylabel('Минимальное расстояние (м)')
    ax3.set_title('Безопасность движения')
    ax3.set_xticks(range(len(exp_ids)))
    ax3.set_xticklabels(exp_ids, rotation=45, ha='right', fontsize=9)
    ax3.grid(True, alpha=0.3, axis='y')
    
    for bar, dist in zip(bars3, min_distances):
        height = bar.get_height()
        ax3.text(bar.get_x() + bar.get_width()/2., height + 0.01,
                f'{dist:.3f}', ha='center', va='bottom', fontsize=8, fontweight='bold')
    
    # График 4: Зависимость площади от скорости
    ax4 = axes[1, 1]
    colors = plt.cm.viridis(np.linspace(0, 1, len(exp_ids)))
    scatter = ax4.scatter(speeds, areas, s=100, c=colors, 
                         edgecolors='black', alpha=0.8)
    ax4.set_xlabel('Скорость (м/с)')
    ax4.set_ylabel('Покрытая площадь (м²)')
    ax4.set_title('Зависимость: Скорость → Площадь')
    ax4.grid(True, alpha=0.3)
    
    # Добавляем тренд
    if len(speeds) > 1:
        z = np.polyfit(speeds, areas, 1)
        p = np.poly1d(z)
        ax4.plot(np.sort(speeds), p(np.sort(speeds)), "r--", alpha=0.5, 
                label=f'Тренд: y={z[0]:.2f}x+{z[1]:.2f}')
        ax4.legend()
    
    for i, (sx, sy, exp_id) in enumerate(zip(speeds, areas, exp_ids)):
        ax4.annotate(exp_id, (sx, sy), textcoords="offset points", 
                    xytext=(0,10), ha='center', fontsize=8)
    
    plt.tight_layout()
    plt.savefig('experiments_analysis.png', dpi=150, bbox_inches='tight')
    print("📊 График 'experiments_analysis.png' сохранен")
    plt.show()

def generate_report(data):
    """Генерация отчёта"""
    if not data:
        return "Нет данных для отчёта"
    
    # Находим лучший эксперимент по площади
    best_exp = max(data, key=lambda x: x.get('area_coverage', 0))
    
    # Статистика
    areas = [exp.get('area_coverage', 0) for exp in data]
    success_rates = [exp.get('avoidance_success_rate', 0) * 100 for exp in data]
    min_distances = [exp.get('min_obstacle_distance', 0) for exp in data]
    collisions = [exp.get('collisions', 0) for exp in data]
    
    # Находим лучший по другим метрикам
    max_success_exp = max(data, key=lambda x: x.get('avoidance_success_rate', 0))
    max_safety_exp = max(data, key=lambda x: x.get('min_obstacle_distance', 0))
    
    report = f"""
{'='*70}
ОТЧЕТ ПО ОПТИМИЗАЦИИ АЛГОРИТМА БРАЙТЕНБЕРГА
{'='*70}

ОБЩАЯ СТАТИСТИКА:
Всего экспериментов: {len(data)}
Средняя покрытая площадь: {statistics.mean(areas):.3f} м²
Максимальная площадь: {max(areas):.3f} м² (эксперимент: {best_exp['experiment_id']})
Минимальная площадь: {min(areas):.3f} м²

Средняя успешность объездов: {statistics.mean(success_rates):.1f}%
Максимальная успешность: {max(success_rates):.1f}% (эксперимент: {max_success_exp['experiment_id']})

Среднее минимальное расстояние: {statistics.mean(min_distances):.3f} м
Максимальная безопасность: {max(min_distances):.3f} м (эксперимент: {max_safety_exp['experiment_id']})

Общее количество столкновений: {sum(collisions)}

{'='*70}
ОПТИМАЛЬНЫЕ ПАРАМЕТРЫ:
{'='*70}
• Лучший эксперимент (по площади): {best_exp['experiment_id']}
• Оптимальная линейная скорость: {best_exp.get('forward_speed', 0):.3f} м/с
• Оптимальная угловая скорость: {best_exp.get('turn_speed', 0):.2f} рад/с
• Оптимальная дистанция остановки: {best_exp.get('stop_distance', 0):.2f} м
• Оптимальная длительность манёвра: {best_exp.get('escape_duration', 0):.1f} с

РЕЗУЛЬТАТЫ ЛУЧШЕГО ЭКСПЕРИМЕНТА:
• Покрытие площади: {best_exp.get('area_coverage', 0):.3f} м²
• Успешность объездов: {best_exp.get('avoidance_success_rate', 0)*100:.1f}%
• Количество объездов: {best_exp.get('total_avoidances', 0)}
• Минимальное расстояние: {best_exp.get('min_obstacle_distance', 0):.3f} м
• Пройденная дистанция: {best_exp.get('total_distance', 0):.2f} м
• Средняя скорость: {best_exp.get('average_speed', 0):.3f} м/с

{'='*70}
РЕКОМЕНДАЦИИ:
{'='*70}
1. Для максимального покрытия используйте скорость {best_exp.get('forward_speed', 0):.2f} м/с
2. Поддерживайте дистанцию остановки не менее {best_exp.get('stop_distance', 0):.2f} м
3. Угловая скорость {best_exp.get('turn_speed', 0):.1f} рад/с обеспечивает хорошую манёвренность
4. Длительность манёвра {best_exp.get('escape_duration', 0):.1f} с оптимальна для объезда препятствий
5. Мониторинг безопасности: минимальное расстояние должно быть > 0.15 м

ВЫВОДЫ:
• Алгоритм Брайтенберга эффективен при правильной настройке параметров
• Слишком высокая скорость может снизить безопасность
• Баланс между скоростью и точностью критически важен
• Регулярный мониторинг метрик позволяет оптимизировать производительность
{'='*70}
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
        print("Сначала запустите эксперименты:")
        print("python3 lab3_fixed_final.py --mode series")
        return
    
    # Вывод сводки
    print(f"\n📊 НАЙДЕНО ЭКСПЕРИМЕНТОВ: {len(data)}")
    for exp in data:
        print(f"  • {exp['experiment_id']}: площадь={exp.get('area_coverage', 0):.3f} м², "
              f"скорость={exp.get('forward_speed', 0):.2f} м/с")
    
    # Создание графиков
    print("\n📈 Создание графиков...")
    create_basic_plots(data)
    
    # Генерация отчёта
    print("\n📝 ГЕНЕРАЦИЯ ОТЧЕТА...")
    report = generate_report(data)
    
    # Сохранение отчёта
    with open('optimization_report_final.txt', 'w', encoding='utf-8') as f:
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
                'avoidance_success_rate': best_exp.get('avoidance_success_rate', 0),
                'min_obstacle_distance': best_exp.get('min_obstacle_distance', 0),
                'total_distance': best_exp.get('total_distance', 0)
            },
            'analysis_date': datetime.now().isoformat()
        }
        
        with open('optimal_parameters_final.json', 'w') as f:
            json.dump(optimal_config, f, indent=2)
        
        print(f"\n💾 Оптимальная конфигурация сохранена в optimal_parameters_final.json")
    
    print(f"\n✅ АНАЛИЗ ЗАВЕРШЕН УСПЕШНО!")
    print(f"📊 Графики сохранены в experiments_analysis.png")
    print(f"📝 Отчет сохранен в optimization_report_final.txt")

if __name__ == '__main__':
    main()

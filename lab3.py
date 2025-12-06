#!/usr/bin/env python3
"""
Лабораторная работа №3: Оптимизация параметров алгоритма Брайтенберга
ИСПРАВЛЕННАЯ ВЕРСИЯ с корректным сохранением в CSV
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import math
import json
import time
import csv
import os
import sys
from datetime import datetime
import threading

class ParameterOptimizer(Node):
    def __init__(self, experiment_params, experiment_duration=15.0):
        super().__init__('parameter_optimizer')
        
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.lidar_callback,
            10)
        
        # Параметры текущего эксперимента
        self.forward_speed = experiment_params['forward_speed']
        self.turn_speed = experiment_params['turn_speed']
        self.stop_distance = experiment_params['stop_distance']
        self.escape_duration = experiment_params['escape_duration']
        
        # Длительность эксперимента
        self.experiment_duration = experiment_duration
        self.start_time = self.get_clock().now()
        
        # Идентификатор эксперимента
        self.experiment_id = experiment_params['experiment_id']
        
        # Состояние робота
        self.state = "MOVING_FORWARD"
        self.turn_direction = "right"
        self.maneuver_start_time = None
        
        # Метрики для сбора
        self.metrics = {
            'experiment_info': {
                'id': self.experiment_id,
                'forward_speed': self.forward_speed,
                'turn_speed': self.turn_speed,
                'stop_distance': self.stop_distance,
                'escape_duration': self.escape_duration,
                'duration': self.experiment_duration
            },
            'avoidances': [],
            'safety': {
                'min_distance': float('inf'),
                'near_misses': 0,
                'collisions': 0
            },
            'mobility': {
                'total_distance': 0.0,
                'avg_speed': 0.0
            },
            'timing': {
                'start_time': datetime.now().isoformat()
            }
        }
        
        # Вспомогательные переменные
        self.last_callback_time = None
        self.total_avoidances = 0
        self.speed_samples = []
        self.is_running = True
        
        self.get_logger().info(
            f"🚀 Эксперимент {self.experiment_id} запущен на {self.experiment_duration} сек"
        )

    def lidar_callback(self, msg):
        if not self.is_running:
            return
            
        current_time = self.get_clock().now()
        experiment_elapsed = (current_time - self.start_time).nanoseconds / 1e9
        
        # Завершение эксперимента по времени
        if experiment_elapsed >= self.experiment_duration:
            self.finish_experiment()
            return
        
        cmd = Twist()
        
        # Получаем минимальное расстояние спереди
        front_min = self.get_front_distance(msg)
        
        # Сбор метрик безопасности
        if front_min < self.metrics['safety']['min_distance']:
            self.metrics['safety']['min_distance'] = front_min
            
        if front_min < self.stop_distance * 0.7:
            self.metrics['safety']['near_misses'] += 1
            
        if front_min < 0.05:
            self.metrics['safety']['collisions'] += 1
        
        # Простой алгоритм Брайтенберга
        if self.state == "MOVING_FORWARD":
            if front_min < self.stop_distance:
                # Начинаем объезд
                self.total_avoidances += 1
                self.state = "TURNING"
                self.maneuver_start_time = current_time
                
                # Выбор направления
                left_dist = self.get_sector_min(msg, 30, 90)
                right_dist = self.get_sector_min(msg, -90, -30)
                
                if left_dist > right_dist:
                    self.turn_direction = "left"
                else:
                    self.turn_direction = "right"
                
                # Запись объезда
                avoidance = {
                    'id': self.total_avoidances,
                    'start_time': experiment_elapsed,
                    'direction': self.turn_direction,
                    'reason': 'left' if left_dist > right_dist else 'right'
                }
                self.metrics['avoidances'].append(avoidance)
                
                # Команда: поворот
                cmd.linear.x = 0.0
                cmd.angular.z = self.turn_speed if self.turn_direction == "left" else -self.turn_speed
                
            else:
                # Движение вперед
                cmd.linear.x = self.forward_speed
                cmd.angular.z = 0.0
                
        elif self.state == "TURNING":
            maneuver_time = (current_time - self.maneuver_start_time).nanoseconds / 1e9
            
            if maneuver_time < self.escape_duration:
                # Продолжаем поворот
                cmd.linear.x = 0.0
                cmd.angular.z = self.turn_speed if self.turn_direction == "left" else -self.turn_speed
            else:
                # Завершаем поворот
                self.state = "MOVING_FORWARD"
                cmd.linear.x = self.forward_speed
                cmd.angular.z = 0.0
                
                # Обновляем запись объезда
                if self.metrics['avoidances']:
                    self.metrics['avoidances'][-1]['duration'] = maneuver_time
                    self.metrics['avoidances'][-1]['successful'] = True
        
        # Сбор метрик мобильности
        self.speed_samples.append(cmd.linear.x)
        if len(self.speed_samples) > 100:
            self.speed_samples = self.speed_samples[-100:]
        
        self.metrics['mobility']['total_distance'] += abs(cmd.linear.x) * 0.1  # ~10Hz
        
        self.publisher.publish(cmd)

    def get_front_distance(self, msg):
        """Получение минимального расстояния спереди"""
        return self.get_sector_min(msg, -30, 30)

    def get_sector_min(self, msg, start_deg, end_deg):
        """Минимальное расстояние в секторе"""
        if len(msg.ranges) == 0:
            return float('inf')
            
        ranges = msg.ranges
        angle_min = msg.angle_min
        angle_max = msg.angle_max
        angle_increment = msg.angle_increment
        
        start_rad = math.radians(start_deg)
        end_rad = math.radians(end_deg)
        
        min_dist = float('inf')
        for i, distance in enumerate(ranges):
            angle = angle_min + i * angle_increment
            if start_rad <= angle <= end_rad or (start_rad > end_rad and (angle >= start_rad or angle <= end_rad)):
                if 0.05 < distance < 10.0:
                    min_dist = min(min_dist, distance)
                    
        return min_dist if min_dist != float('inf') else 10.0

    def finish_experiment(self):
        """Завершение эксперимента"""
        if not self.is_running:
            return
            
        self.is_running = False
        
        # Остановка робота
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.angular.z = 0.0
        self.publisher.publish(cmd)
        
        # Расчет финальных метрик
        if self.speed_samples:
            valid_speeds = [s for s in self.speed_samples if s > 0]
            if valid_speeds:
                self.metrics['mobility']['avg_speed'] = sum(valid_speeds) / len(valid_speeds)
        
        # Расчет успешности объездов
        successful_avoidances = sum(1 for a in self.metrics['avoidances'] if a.get('successful', False))
        success_rate = successful_avoidances / len(self.metrics['avoidances']) if self.metrics['avoidances'] else 0
        
        # Оценка покрытой площади (простая модель)
        estimated_area = self.metrics['mobility']['total_distance'] * 0.3  # примерная ширина охвата
        
        # Сохранение результатов
        summary = {
            'experiment_id': self.experiment_id,
            'forward_speed': self.forward_speed,
            'turn_speed': self.turn_speed,
            'stop_distance': self.stop_distance,
            'escape_duration': self.escape_duration,
            'area_coverage': estimated_area,
            'total_avoidances': len(self.metrics['avoidances']),
            'avoidance_success_rate': success_rate,
            'min_obstacle_distance': self.metrics['safety']['min_distance'],
            'near_misses': self.metrics['safety']['near_misses'],
            'collisions': self.metrics['safety']['collisions'],
            'total_distance': self.metrics['mobility']['total_distance'],
            'average_speed': self.metrics['mobility']['avg_speed']
        }
        
        self.save_results(summary)
        
        self.get_logger().info(
            f"✅ Эксперимент {self.experiment_id} завершен: "
            f"площадь={estimated_area:.2f}м², объездов={len(self.metrics['avoidances'])}, "
            f"успешность={success_rate:.1%}"
        )
        
        # Сигнал для завершения spin
        self.destroy_node()

    def save_results(self, summary):
        """Сохранение результатов эксперимента"""
        # Сохранение в JSON
        json_filename = f"experiment_{self.experiment_id}_results.json"
        with open(json_filename, 'w') as f:
            json.dump({
                'full_metrics': self.metrics,
                'summary': summary
            }, f, indent=2)
        
        # Сохранение в CSV для сводной таблицы
        csv_filename = "all_experiments_summary.csv"
        
        # Определяем заголовки (ТОЛЬКО те, которые у нас есть)
        headers = [
            'experiment_id', 
            'forward_speed', 
            'turn_speed', 
            'stop_distance', 
            'escape_duration',
            'area_coverage', 
            'total_avoidances', 
            'avoidance_success_rate',
            'min_obstacle_distance', 
            'near_misses', 
            'collisions',
            'total_distance', 
            'average_speed'
        ]
        
        # Проверяем существование файла и читаем существующие данные
        existing_data = []
        file_exists = os.path.isfile(csv_filename)
        
        if file_exists:
            try:
                with open(csv_filename, 'r') as f:
                    reader = csv.DictReader(f)
                    existing_data = list(reader)
            except Exception as e:
                print(f"⚠️ Ошибка чтения CSV файла: {e}")
                existing_data = []
        
        # Проверяем, есть ли уже этот эксперимент в CSV
        experiment_exists = False
        for row in existing_data:
            if row.get('experiment_id') == summary['experiment_id']:
                experiment_exists = True
                break
        
        if not experiment_exists:
            # Добавляем новый эксперимент
            with open(csv_filename, 'a', newline='') as f:
                writer = csv.DictWriter(f, fieldnames=headers)
                
                # Если файл новый или пустой, записываем заголовок
                if not file_exists or os.path.getsize(csv_filename) == 0:
                    writer.writeheader()
                
                # Записываем данные
                writer.writerow(summary)
            
            self.get_logger().info(f"💾 Результаты сохранены в {json_filename} и добавлены в {csv_filename}")
        else:
            self.get_logger().info(f"💾 Результаты сохранены в {json_filename} (уже есть в {csv_filename})")

def run_single_experiment(params, duration=15.0):
    """Запуск одного эксперимента"""
    print(f"\n{'='*60}")
    print(f"🚀 ЗАПУСК ЭКСПЕРИМЕНТА: {params['experiment_id']}")
    print(f"Длительность: {duration} сек")
    print(f"Параметры: speed={params['forward_speed']:.2f}, "
          f"turn={params['turn_speed']:.1f}, "
          f"stop={params['stop_distance']:.2f}")
    print(f"{'='*60}")
    
    rclpy.init()
    node = ParameterOptimizer(params, duration)
    
    # Запуск в отдельном потоке для контроля времени
    def spin_node():
        rclpy.spin(node)
    
    spin_thread = threading.Thread(target=spin_node)
    spin_thread.start()
    
    try:
        # Ждем завершения эксперимента
        spin_thread.join(timeout=duration + 5)  # +5 секунд на завершение
        
        if spin_thread.is_alive():
            print("⚠️ Эксперимент не завершился вовремя, принудительная остановка...")
            node.destroy_node()
            rclpy.shutdown()
            spin_thread.join(timeout=2.0)
            
    except KeyboardInterrupt:
        print("\n🛑 Эксперимент прерван пользователем")
        node.destroy_node()
        rclpy.shutdown()
        spin_thread.join(timeout=2.0)
    
    # Короткая пауза
    time.sleep(1.0)
    
    return True

def create_experiment_plan():
    """Создание плана экспериментов"""
    experiments = []
    
    # СОКРАЩЕННЫЙ ПЛАН: 4 эксперимента для быстрого тестирования
    print("📋 СОЗДАНИЕ ПЛАНА ЭКСПЕРИМЕНТОВ (4 эксперимента)")
    
    # 1. Тестирование скорости
    experiments.append({
        'experiment_id': 'test_slow',
        'forward_speed': 0.10,
        'turn_speed': 1.5,
        'stop_distance': 0.3,
        'escape_duration': 1.5
    })
    
    experiments.append({
        'experiment_id': 'test_medium',
        'forward_speed': 0.15,
        'turn_speed': 1.5,
        'stop_distance': 0.3,
        'escape_duration': 1.5
    })
    
    experiments.append({
        'experiment_id': 'test_fast',
        'forward_speed': 0.20,
        'turn_speed': 1.5,
        'stop_distance': 0.3,
        'escape_duration': 1.5
    })
    
    # 2. Тестирование угловой скорости
    experiments.append({
        'experiment_id': 'test_turn',
        'forward_speed': 0.15,
        'turn_speed': 2.0,
        'stop_distance': 0.3,
        'escape_duration': 1.5
    })
    
    return experiments

def run_experiment_series():
    """Запуск серии экспериментов"""
    experiments = create_experiment_plan()
    
    print(f"\n📊 ПЛАН ЭКСПЕРИМЕНТОВ:")
    print("-" * 40)
    for i, exp in enumerate(experiments, 1):
        print(f"{i:2d}. {exp['experiment_id']:15s} "
              f"speed={exp['forward_speed']:.2f} "
              f"turn={exp['turn_speed']:.1f} "
              f"stop={exp['stop_distance']:.2f}")
    
    print(f"\n{'='*60}")
    print(f"🚀 НАЧАЛО СЕРИИ ИЗ {len(experiments)} ЭКСПЕРИМЕНТОВ")
    print(f"Каждый эксперимент длится 15 секунд")
    print(f"{'='*60}")
    
    for i, exp_params in enumerate(experiments, 1):
        success = run_single_experiment(exp_params, duration=15.0)
        
        if not success:
            print(f"❌ Эксперимент {i} завершился с ошибкой")
            break
        
        if i < len(experiments):
            print(f"\n⏳ Пауза 2 секунды перед следующим экспериментом...")
            time.sleep(2.0)
    
    print(f"\n{'='*60}")
    print(f"✅ СЕРИЯ ЭКСПЕРИМЕНТОВ ЗАВЕРШЕНА")
    print(f"📊 Проведено: {len(experiments)} экспериментов")
    print(f"📈 Для анализа запустите: python3 analyze_results.py")
    print(f"{'='*60}")

def create_clean_csv():
    """Создание чистого CSV файла из существующих JSON файлов"""
    print("🔄 СОЗДАНИЕ ЧИСТОГО CSV ФАЙЛА ИЗ JSON...")
    
    import glob
    json_files = glob.glob("experiment_*_results.json") + glob.glob("*_results.json")
    
    if not json_files:
        print("❌ Не найдены JSON файлы с результатами!")
        return
    
    print(f"📁 Найдено {len(json_files)} JSON файлов")
    
    # Определяем заголовки
    headers = [
        'experiment_id', 
        'forward_speed', 
        'turn_speed', 
        'stop_distance', 
        'escape_duration',
        'area_coverage', 
        'total_avoidances', 
        'avoidance_success_rate',
        'min_obstacle_distance', 
        'near_misses', 
        'collisions',
        'total_distance', 
        'average_speed'
    ]
    
    all_data = []
    
    # Читаем данные из JSON файлов
    for json_file in json_files:
        try:
            with open(json_file, 'r') as f:
                data = json.load(f)
            
            summary = data.get('summary', {})
            
            # Извлекаем данные
            row_data = {
                'experiment_id': summary.get('experiment_id', os.path.basename(json_file).replace('.json', '')),
                'forward_speed': summary.get('forward_speed', 0),
                'turn_speed': summary.get('turn_speed', 0),
                'stop_distance': summary.get('stop_distance', 0),
                'escape_duration': summary.get('escape_duration', 0),
                'area_coverage': summary.get('area_coverage', 0),
                'total_avoidances': summary.get('total_avoidances', 0),
                'avoidance_success_rate': summary.get('avoidance_success_rate', 0),
                'min_obstacle_distance': summary.get('min_obstacle_distance', 0),
                'near_misses': summary.get('near_misses', 0),
                'collisions': summary.get('collisions', 0),
                'total_distance': summary.get('total_distance', 0),
                'average_speed': summary.get('average_speed', 0)
            }
            
            # Для файла speed_1 (особый формат)
            if 'parameters' in summary:
                params = summary.get('parameters', {})
                performance = summary.get('performance_metrics', {})
                row_data.update({
                    'forward_speed': params.get('forward_speed', 0),
                    'turn_speed': params.get('turn_speed', 0),
                    'stop_distance': params.get('stop_distance', 0),
                    'escape_duration': params.get('escape_duration', 0),
                    'area_coverage': performance.get('area_coverage', 0),
                    'total_avoidances': performance.get('total_avoidances', 0),
                    'avoidance_success_rate': performance.get('avoidance_success_rate', 0),
                    'min_obstacle_distance': performance.get('min_obstacle_distance', 0),
                    'near_misses': performance.get('near_misses', 0),
                    'collisions': performance.get('collisions', 0),
                    'total_distance': performance.get('total_distance', 0),
                    'average_speed': performance.get('average_speed', 0)
                })
            
            all_data.append(row_data)
            print(f"✅ Извлечено: {row_data['experiment_id']}")
            
        except Exception as e:
            print(f"❌ Ошибка при чтении {json_file}: {e}")
    
    # Записываем в CSV
    if all_data:
        with open('all_experiments_summary.csv', 'w', newline='') as f:
            writer = csv.DictWriter(f, fieldnames=headers)
            writer.writeheader()
            writer.writerows(all_data)
        
        print(f"\n💾 Создан чистый CSV файл: all_experiments_summary.csv")
        print(f"📊 Всего записей: {len(all_data)}")
    else:
        print("❌ Не удалось извлечь данные из JSON файлов!")

def main():
    import argparse
    
    parser = argparse.ArgumentParser(description='Оптимизация параметров алгоритма Брайтенберга')
    parser.add_argument('--mode', type=str, default='series',
                       choices=['series', 'single', 'quick', 'clean_csv'],
                       help='Режим работы: series (серия), single (один), quick (быстрый тест), clean_csv (очистить CSV)')
    
    args = parser.parse_args()
    
    if args.mode == 'series':
        run_experiment_series()
    elif args.mode == 'single':
        # Один тестовый эксперимент
        params = {
            'experiment_id': 'single_test',
            'forward_speed': 0.15,
            'turn_speed': 1.5,
            'stop_distance': 0.3,
            'escape_duration': 1.5
        }
        run_single_experiment(params, duration=10.0)
    elif args.mode == 'quick':
        # Быстрый тест - 2 эксперимента по 10 секунд
        print("⚡ БЫСТРЫЙ ТЕСТ (2 эксперимента по 10 секунд)")
        
        experiments = [
            {
                'experiment_id': 'quick_slow',
                'forward_speed': 0.10,
                'turn_speed': 1.5,
                'stop_distance': 0.3,
                'escape_duration': 1.5
            },
            {
                'experiment_id': 'quick_fast',
                'forward_speed': 0.20,
                'turn_speed': 1.5,
                'stop_distance': 0.3,
                'escape_duration': 1.5
            }
        ]
        
        for exp in experiments:
            run_single_experiment(exp, duration=10.0)
            time.sleep(1.0)
        
        print("\n✅ Быстрый тест завершен!")
    elif args.mode == 'clean_csv':
        # Очистка и пересоздание CSV файла
        create_clean_csv()

if __name__ == '__main__':
    main()

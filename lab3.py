#!/usr/bin/env python3
"""
Лабораторная работа №3: Оптимизация параметров алгоритма Брайтенберга
ИСПРАВЛЕННАЯ ВЕРСИЯ для корректной работы с analyze_results.py
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
import numpy as np

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
        self.current_avoidance_id = None
        
        # Для отслеживания пути и расчёта площади
        self.robot_positions = []  # (x, y, время)
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_theta = 0.0  # текущий угол
        self.position_update_time = 0
        self.last_position_time = time.time()
        
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
                'avg_speed': 0.0,
                'max_speed': 0.0,
                'area_coverage': 0.0
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
        self.experiment_finished = False
        self.experiment_start_time = time.time()
        
        self.get_logger().info(
            f"🚀 Эксперимент {self.experiment_id} запущен на {self.experiment_duration} сек"
        )
        self.get_logger().info(
            f"⚙️ Параметры: скорость={self.forward_speed:.2f}, "
            f"поворот={self.turn_speed:.1f}, стоп={self.stop_distance:.2f}"
        )

    def update_position(self, linear_speed, angular_speed):
        """Обновление позиции робота для расчёта покрытия"""
        current_time = time.time()
        dt = current_time - self.last_position_time
        
        if dt > 0.5:  # Сохраняем позицию каждые 0.5 секунды
            # Обновляем угол
            self.current_theta += angular_speed * dt * 0.5  # Меньший коэффициент для стабильности
            
            # Обновляем позицию
            if abs(angular_speed) < 0.01:  # Движение почти прямо
                distance = linear_speed * dt
                self.current_x += distance * math.cos(self.current_theta)
                self.current_y += distance * math.sin(self.current_theta)
            else:  # Поворот
                # При повороте движение меньше
                distance = linear_speed * dt * 0.3
                self.current_x += distance * math.cos(self.current_theta)
                self.current_y += distance * math.sin(self.current_theta)
            
            self.robot_positions.append((self.current_x, self.current_y, current_time))
            self.last_position_time = current_time

    def calculate_area_coverage(self):
        """Расчёт покрытой площади на основе пути робота"""
        if len(self.robot_positions) < 2:
            return 0.0
        
        # Извлекаем координаты
        points = np.array([(p[0], p[1]) for p in self.robot_positions])
        
        if len(points) < 2:
            return 0.0
        
        # Рассчитываем bounding box (минимальный прямоугольник, содержащий все точки)
        min_x, min_y = np.min(points, axis=0)
        max_x, max_y = np.max(points, axis=0)
        
        # Площадь bounding box
        width = max_x - min_x
        height = max_y - min_y
        
        # Если точки выстроены в линию (высота очень мала), добавляем ширину робота
        if height < 0.1:
            height = 0.5  # ширина робота
        
        # Рассчитываем примерную площадь
        area = width * height
        
        # Корректируем площадь на основе пройденного расстояния
        if area < 0.01:  # Если площадь очень маленькая
            # Используем пройденное расстояние * ширину охвата
            area = self.metrics['mobility']['total_distance'] * 0.5
        
        # Ограничиваем разумными значениями
        area = max(0.1, min(area, 5.0))
        
        return area

    def lidar_callback(self, msg):
        if not self.is_running or self.experiment_finished:
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
                self.current_avoidance_id = self.total_avoidances
                
                # Выбор направления
                left_dist = self.get_sector_min(msg, 30, 90)
                right_dist = self.get_sector_min(msg, -90, -30)
                
                if left_dist > right_dist:
                    self.turn_direction = "left"
                    reason = "left"
                else:
                    self.turn_direction = "right"
                    reason = "right"
                
                # Запись объезда
                avoidance = {
                    'id': self.total_avoidances,
                    'start_time': experiment_elapsed,
                    'direction': self.turn_direction,
                    'reason': reason,
                    'duration': None,
                    'successful': False
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
                    self.current_avoidance_id = None
        
        # Сбор метрик мобильности
        self.speed_samples.append(cmd.linear.x)
        if len(self.speed_samples) > 100:
            self.speed_samples = self.speed_samples[-100:]
        
        # Обновление пройденного расстояния
        dt = 0.1  # Примерное время между вызовами (10 Гц)
        self.metrics['mobility']['total_distance'] += abs(cmd.linear.x) * dt
        
        # Обновление позиции для расчёта покрытия
        self.update_position(cmd.linear.x, cmd.angular.z)
        
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
        if self.experiment_finished:
            return
            
        self.experiment_finished = True
        self.is_running = False
        
        # Завершаем текущий объезд, если он активен
        if self.state == "TURNING" and self.current_avoidance_id:
            current_time = self.get_clock().now()
            if self.maneuver_start_time:
                maneuver_time = (current_time - self.maneuver_start_time).nanoseconds / 1e9
                for avoidance in self.metrics['avoidances']:
                    if avoidance['id'] == self.current_avoidance_id:
                        avoidance['duration'] = maneuver_time
                        avoidance['successful'] = True
                        break
        
        # Остановка робота
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.angular.z = 0.0
        self.publisher.publish(cmd)
        
        # Расчет финальных метрик
        if self.speed_samples:
            valid_speeds = [abs(s) for s in self.speed_samples if s != 0]
            if valid_speeds:
                self.metrics['mobility']['avg_speed'] = sum(valid_speeds) / len(valid_speeds)
                self.metrics['mobility']['max_speed'] = max(valid_speeds)
        
        # Расчет покрытой площади
        area_coverage = self.calculate_area_coverage()
        self.metrics['mobility']['area_coverage'] = area_coverage
        
        # Расчет успешности объездов
        successful_avoidances = sum(1 for a in self.metrics['avoidances'] if a.get('successful', False))
        total_avoidances = len(self.metrics['avoidances'])
        success_rate = successful_avoidances / total_avoidances if total_avoidances > 0 else 0.0
        
        # Время окончания
        self.metrics['timing']['end_time'] = datetime.now().isoformat()
        
        # Создание сводки для CSV
        summary = {
            'experiment_id': self.experiment_id,
            'forward_speed': self.forward_speed,
            'turn_speed': self.turn_speed,
            'stop_distance': self.stop_distance,
            'escape_duration': self.escape_duration,
            'area_coverage': area_coverage,
            'total_avoidances': total_avoidances,
            'avoidance_success_rate': success_rate,
            'min_obstacle_distance': self.metrics['safety']['min_distance'],
            'near_misses': self.metrics['safety']['near_misses'],
            'collisions': self.metrics['safety']['collisions'],
            'total_distance': self.metrics['mobility']['total_distance'],
            'average_speed': self.metrics['mobility']['avg_speed']
        }
        
        self.save_results(summary)
        
        print(f"\n✅ Эксперимент {self.experiment_id} завершен:")
        print(f"   📏 Площадь покрытия: {area_coverage:.3f} м²")
        print(f"   🚗 Количество объездов: {total_avoidances}")
        print(f"   ✅ Успешность объездов: {success_rate:.1%}")
        print(f"   🛡️  Минимальное расстояние: {self.metrics['safety']['min_distance']:.3f} м")
        print(f"   📏 Пройденное расстояние: {self.metrics['mobility']['total_distance']:.2f} м")
        print(f"   🚀 Средняя скорость: {self.metrics['mobility']['avg_speed']:.3f} м/с")
        
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
        
        # Проверяем существование файла и читаем существующие данные
        existing_data = []
        file_exists = os.path.isfile(csv_filename)
        
        if file_exists:
            try:
                with open(csv_filename, 'r') as f:
                    reader = csv.DictReader(f)
                    for row in reader:
                        # Конвертируем строки в числа
                        numeric_row = {}
                        for key, value in row.items():
                            if key == 'experiment_id':
                                numeric_row[key] = value
                            else:
                                try:
                                    numeric_row[key] = float(value)
                                except:
                                    numeric_row[key] = 0.0
                        existing_data.append(numeric_row)
            except Exception as e:
                print(f"⚠️ Ошибка чтения CSV файла: {e}")
        
        # Удаляем старую запись этого эксперимента, если она есть
        existing_data = [row for row in existing_data if row.get('experiment_id') != summary['experiment_id']]
        
        # Добавляем новую запись
        existing_data.append(summary)
        
        # Сортируем по ID эксперимента
        existing_data.sort(key=lambda x: x['experiment_id'])
        
        # Записываем обратно
        with open(csv_filename, 'w', newline='') as f:
            writer = csv.DictWriter(f, fieldnames=headers)
            writer.writeheader()
            writer.writerows(existing_data)
        
        print(f"   💾 Результаты сохранены в {json_filename} и обновлены в {csv_filename}")

# Глобальная переменная для управления контекстом ROS
ros_initialized = False

def run_single_experiment(params, duration=15.0):
    """Запуск одного эксперимента"""
    print(f"\n{'='*60}")
    print(f"🚀 ЗАПУСК ЭКСПЕРИМЕНТА: {params['experiment_id']}")
    print(f"Длительность: {duration} сек")
    print(f"Параметры: speed={params['forward_speed']:.2f}, "
          f"turn={params['turn_speed']:.1f}, "
          f"stop={params['stop_distance']:.2f}")
    print(f"{'='*60}")
    
    global ros_initialized
    
    try:
        if not ros_initialized:
            rclpy.init()
            ros_initialized = True
        
        node = ParameterOptimizer(params, duration)
        
        # Запуск в отдельном потоке для контроля времени
        def spin_node():
            rclpy.spin(node)
        
        spin_thread = threading.Thread(target=spin_node)
        spin_thread.start()
        
        # Ждем завершения эксперимента
        spin_thread.join(timeout=duration + 3)  # +3 секунды на завершение
        
        if spin_thread.is_alive():
            print("⚠️ Эксперимент не завершился вовремя, принудительная остановка...")
            node.destroy_node()
            rclpy.shutdown()
            ros_initialized = False
            spin_thread.join(timeout=2.0)
            
    except KeyboardInterrupt:
        print("\n🛑 Эксперимент прерван пользователем")
        node.destroy_node()
        rclpy.shutdown()
        ros_initialized = False
        spin_thread.join(timeout=2.0)
    except Exception as e:
        print(f"\n❌ Ошибка при запуске эксперимента: {e}")
        return False
    
    # Короткая пауза
    time.sleep(1.0)
    
    return True

def create_experiment_plan():
    """Создание плана экспериментов"""
    experiments = []
    
    print("📋 СОЗДАНИЕ ПЛАНА ЭКСПЕРИМЕНТОВ")
    
    # Основные эксперименты для анализа
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
    print("-" * 50)
    for i, exp in enumerate(experiments, 1):
        print(f"{i:2d}. {exp['experiment_id']:15s} "
              f"speed={exp['forward_speed']:.2f} "
              f"turn={exp['turn_speed']:.1f} "
              f"stop={exp['stop_distance']:.2f}")
    
    print(f"\n{'='*60}")
    print(f"🚀 НАЧАЛО СЕРИИ ИЗ {len(experiments)} ЭКСПЕРИМЕНТОВ")
    print(f"Каждый эксперимент длится 15 секунд")
    print(f"{'='*60}")
    
    # Очищаем старый CSV файл
    if os.path.exists("all_experiments_summary.csv"):
        os.remove("all_experiments_summary.csv")
        print("🗑️  Старый CSV файл удален")
    
    for i, exp_params in enumerate(experiments, 1):
        print(f"\n▶️  Эксперимент {i}/{len(experiments)}")
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
    json_files = glob.glob("experiment_*_results.json")
    
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
    for json_file in sorted(json_files):
        try:
            with open(json_file, 'r') as f:
                data = json.load(f)
            
            summary = data.get('summary', {})
            
            # Извлекаем данные в правильном формате
            row_data = {
                'experiment_id': summary.get('experiment_id', 'unknown'),
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
            
            all_data.append(row_data)
            print(f"✅ Извлечено: {row_data['experiment_id']}")
            
        except Exception as e:
            print(f"❌ Ошибка при чтении {json_file}: {e}")
    
    # Сортируем по ID эксперимента
    all_data.sort(key=lambda x: x['experiment_id'])
    
    # Записываем в CSV
    if all_data:
        with open('all_experiments_summary.csv', 'w', newline='') as f:
            writer = csv.DictWriter(f, fieldnames=headers)
            writer.writeheader()
            writer.writerows(all_data)
        
        print(f"\n💾 Создан чистый CSV файл: all_experiments_summary.csv")
        print(f"📊 Всего записей: {len(all_data)}")
        
        # Показываем сводку
        print(f"\n📋 СВОДКА ДАННЫХ:")
        for row in all_data:
            print(f"  {row['experiment_id']}: площадь={row['area_coverage']:.3f}м², "
                  f"объездов={row['total_avoidances']}, успешность={row['avoidance_success_rate']:.1%}")
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
